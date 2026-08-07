use crate::{
    Result,
    errors::Error,
    geom_impls::{self, ArcConnector, ExtrudeConnector, LineConnector, RevoluteConnector},
    topo_traits::*,
};
use monstertruck_geometry::prelude::*;
use monstertruck_topology::*;
const PI: Rad<f64> = Rad(std::f64::consts::PI);

/// Sweep angle for revolve operations.
#[derive(Debug, Clone, Copy)]
pub enum SweepAngle {
    /// Partial revolution by the given angle.
    Partial(Rad<f64>),
    /// Full 360° closed revolution.
    Closed,
}

type Vertex = monstertruck_topology::Vertex<Point3>;
type Edge<C> = monstertruck_topology::Edge<Point3, C>;
type Wire<C> = monstertruck_topology::Wire<Point3, C>;
type Face<C, S> = monstertruck_topology::Face<Point3, C, S>;
type Shell<C, S> = monstertruck_topology::Shell<Point3, C, S>;

/// Creates and returns a vertex by a three dimensional point.
/// # Examples
/// ```
/// use monstertruck_modeling::*;
///
/// // put a vertex
/// let vertex = builder::vertex((1.0, 2.0, 3.0));
/// # assert_eq!(vertex.point(), Point3::new(1.0, 2.0, 3.0));
/// ```
#[inline(always)]
pub fn vertex<P: Into<Point3>>(p: P) -> Vertex { Vertex::new(p.into()) }

/// Creates and returns vertices by three dimensional points.
/// # Examples
/// ```
/// use monstertruck_modeling::*;
///
/// // put vertices of a unit cube
/// let vertices = builder::vertices([
///     (0.0, 0.0, 0.0),
///     (1.0, 0.0, 0.0),
///     (0.0, 1.0, 0.0),
///     (0.0, 0.0, 1.0),
///     (0.0, 1.0, 1.0),
///     (1.0, 0.0, 1.0),
///     (1.0, 1.0, 0.0),
///     (1.0, 1.0, 1.0),
/// ]);
/// # assert_eq!(vertices[3].point(), Point3::new(0.0, 0.0, 1.0));
/// ```
#[inline(always)]
pub fn vertices<P: Into<Point3>>(points: impl IntoIterator<Item = P>) -> Vec<Vertex> {
    points.into_iter().map(|p| Vertex::new(p.into())).collect()
}

/// Returns a line from `vertex0` to `vertex1`.
/// # Examples
/// ```
/// use monstertruck_modeling::*;
///
/// // draw a line
/// let vertex0: Vertex = builder::vertex(Point3::new(1.0, 2.0, 3.0));
/// let vertex1: Vertex = builder::vertex(Point3::new(6.0, 5.0, 4.0));
/// let line: Edge = builder::line(&vertex0, &vertex1);
/// # let curve = line.oriented_curve();
/// # let pt0 = Point3::new(1.0, 2.0, 3.0);
/// # let pt1 = Point3::new(6.0, 5.0, 4.0);
/// # const N: usize = 10;
/// # for i in 0..=N {
/// #     let t = i as f64 / N as f64;
/// #     assert!(curve.subs(t).near2(&(pt0 + t * (pt1 - pt0))));
/// # }
/// ```
pub fn line<C>(vertex0: &Vertex, vertex1: &Vertex) -> Edge<C>
where Line<Point3>: ToSameGeometry<C> {
    let pt0 = vertex0.point();
    let pt1 = vertex1.point();
    Edge::new(vertex0, vertex1, Line(pt0, pt1).to_same_geometry())
}

/// Additional constraint that, together with the two endpoints, determines a circular arc.
///
/// Both shapes -- through-point and start-tangent -- pick out the same kind
/// of curve (a single circular arc that travels less than a full
/// revolution from `vertex0` to `vertex1`) but supply the missing degree
/// of freedom in different ways:
///
/// - [`CircularArcConstraint::ThroughPoint`] names a third point on the arc.
///   The plane and radius follow from circumscribing the three points.
/// - [`CircularArcConstraint::StartTangent`] names the tangent direction
///   at `vertex0`. The plane is spanned by the tangent and the chord
///   `vertex1 - vertex0`; the radius is whatever value makes an arc with
///   that start tangent reach `vertex1`.
///
/// The enum implements `From<Point3>` and `From<Vector3>` so existing
/// call sites that already pass a [`Point3`] keep compiling.
#[derive(Clone, Copy, Debug, derive_more::From)]
pub enum CircularArcConstraint {
    /// A point that the arc must pass through.
    ThroughPoint(Point3),
    /// A tangent vector that the arc must have at the start point.
    StartTangent(Vector3),
}

/// Returns a circle arc from `vertex0` to `vertex1` satisfying `constraint`.
///
/// `constraint` is either a [`Point3`] the arc must pass through or a
/// [`Vector3`] the arc must have as its tangent at `vertex0`. See
/// [`CircularArcConstraint`] for the precise interpretation.
///
/// # Panics
///
/// Panics if the inputs are degenerate. Specifically, when `constraint`
/// is [`CircularArcConstraint::StartTangent`], the tangent must be
/// non-zero and not parallel to the chord between the endpoints. Use
/// [`try_circle_arc`] to handle these cases as recoverable errors.
///
/// # Examples
/// ```
/// use monstertruck_modeling::*;
///
/// // The upper unit semicircle, specified by a through-point.
/// let vertex0 = builder::vertex(Point3::new(1.0, 0.0, 0.0));
/// let vertex1 = builder::vertex(Point3::new(-1.0, 0.0, 0.0));
/// let semi_circle = builder::circle_arc(&vertex0, &vertex1, Point3::new(0.0, 1.0, 0.0));
/// # let curve = match semi_circle.oriented_curve() {
/// #       Curve::NurbsCurve(curve) => curve,
/// #       _ => unreachable!(),
/// # };
/// # const N: usize = 10;
/// # for i in 0..=N {
/// #       let t = curve.knot_vector()[0] + curve.knot_vector().range_length() * i as f64 / N as f64;
/// #       assert!(curve.subs(t).to_vec().magnitude().near(&1.0));
/// # }
/// ```
/// ```
/// use monstertruck_modeling::*;
///
/// // A quarter of the unit circle, specified by the start tangent direction.
/// let vertex0 = builder::vertex(Point3::new(1.0, 0.0, 0.0));
/// let vertex1 = builder::vertex(Point3::new(0.0, 1.0, 0.0));
/// let tangent = Vector3::new(0.0, 1.0, 0.0);
/// let quarter = builder::circle_arc(&vertex0, &vertex1, tangent);
/// # let curve = match quarter.oriented_curve() {
/// #       Curve::NurbsCurve(curve) => curve,
/// #       _ => unreachable!(),
/// # };
/// # const N: usize = 10;
/// # for i in 0..=N {
/// #       let t = curve.knot_vector()[0] + curve.knot_vector().range_length() * i as f64 / N as f64;
/// #       assert!(curve.evaluate(t).to_vec().magnitude().near(&1.0));
/// # }
/// ```
pub fn circle_arc<C>(
    vertex0: &Vertex,
    vertex1: &Vertex,
    constraint: impl Into<CircularArcConstraint>,
) -> Edge<C>
where
    Processor<TrimmedCurve<UnitCircle<Point3>>, Matrix4>: ToSameGeometry<C>,
{
    try_circle_arc(vertex0, vertex1, constraint).expect("degenerate circular-arc constraint.")
}

/// Fallible variant of [`circle_arc`].
///
/// Returns:
/// - [`Error::DegenerateCircularArcTangent`] if the tangent is zero or
///   near-zero.
/// - [`Error::CircularArcTangentParallelToChord`] if the tangent is
///   parallel to the chord between the endpoints.
pub fn try_circle_arc<C>(
    vertex0: &Vertex,
    vertex1: &Vertex,
    constraint: impl Into<CircularArcConstraint>,
) -> std::result::Result<Edge<C>, Error>
where
    Processor<TrimmedCurve<UnitCircle<Point3>>, Matrix4>: ToSameGeometry<C>,
{
    let pt0 = vertex0.point();
    let pt1 = vertex1.point();
    let curve = match constraint.into() {
        CircularArcConstraint::ThroughPoint(transit) => {
            geom_impls::circle_arc_by_three_points(pt0, pt1, transit)
        }
        CircularArcConstraint::StartTangent(tangent) => {
            geom_impls::try_circle_arc_by_start_tangent(pt0, pt1, tangent)?
        }
    };
    Ok(Edge::new(vertex0, vertex1, curve.to_same_geometry()))
}

/// Returns a Bezier curve from `vertex0` to `vertex1` with inter control points `inter_points`.
/// # Examples
/// ```
/// use monstertruck_modeling::*;
///
/// // draw a Bezier curve
/// let vertex0 = builder::vertex(Point3::origin());
/// let vertex1 = builder::vertex(Point3::new(3.0, 0.0, 0.0));
/// let inter_points = vec![Point3::new(1.0, 1.0, 0.0), Point3::new(2.0, -1.0, 0.0)];
/// let bezier: Edge = builder::bezier(&vertex0, &vertex1, inter_points);
/// # let curve = bezier.oriented_curve();
/// # const N: usize = 10;
/// # for i in 0..=N {
/// #       let t = i as f64 / N as f64;
/// #       let pt = Point3::new(t * 3.0, 6.0 * t * t * t - 9.0 * t * t + 3.0 * t, 0.0);
/// #       assert!(curve.subs(t).near(&pt));
/// # }
/// ```
pub fn bezier<C>(vertex0: &Vertex, vertex1: &Vertex, mut inter_points: Vec<Point3>) -> Edge<C>
where BsplineCurve<Point3>: ToSameGeometry<C> {
    let pt0 = vertex0.point();
    let pt1 = vertex1.point();
    let mut control_points = vec![pt0];
    control_points.append(&mut inter_points);
    control_points.push(pt1);
    let knot_vec = KnotVector::bezier_knot(control_points.len() - 1);
    let curve = BsplineCurve::new(knot_vec, control_points);
    Edge::new(vertex0, vertex1, curve.to_same_geometry())
}

/// Returns a homotopic face from `edge0` to `edge1`.
/// # Examples
/// ```
/// use monstertruck_modeling::*;
///
/// // homotopy between skew lines
/// let v0 = builder::vertex(Point3::new(0.0, 0.0, 0.0));
/// let v1 = builder::vertex(Point3::new(1.0, 0.0, 0.0));
/// let v2 = builder::vertex(Point3::new(0.0, 1.0, 0.0));
/// let v3 = builder::vertex(Point3::new(0.0, 1.0, 1.0));
/// let line0 = builder::line(&v0, &v1);
/// let line1 = builder::line(&v2, &v3);
/// let homotopy: Face = builder::homotopy(&line0, &line1);
/// # let surface = homotopy.oriented_surface();
/// # const N: usize = 10;
/// # for i in 0..=N {
/// #       for j in 0..=N {
/// #           let s = i as f64 / N as f64;
/// #           let t = j as f64 / N as f64;
/// #           let pt = Point3::new(s * (1.0 - t), t, s * t);
/// #           assert!(surface.subs(s, t).near(&pt));
/// #       }
/// # }
/// ```
pub fn homotopy<C, S>(edge0: &Edge<C>, edge1: &Edge<C>) -> Face<C, S>
where
    C: Invertible,
    Line<Point3>: ToSameGeometry<C>,
    HomotopySurface<C, C>: ToSameGeometry<S>, {
    let wire = wire![
        edge0.clone(),
        line(edge0.back(), edge1.back()),
        edge1.inverse(),
        line(edge1.front(), edge0.front()),
    ];
    let curve0 = edge0.oriented_curve();
    let curve1 = edge1.oriented_curve();
    let homotopy = HomotopySurface::new(curve0, curve1);
    Face::new_unchecked(vec![wire], homotopy.to_same_geometry())
}

/// Returns a homotopic shell from `wire0` to `wire1`.
/// # Examples
/// ```
/// # fn main() -> anyhow::Result<()> {
/// // connecting two squares.
/// use monstertruck_modeling::*;
///
/// let v00 = builder::vertex(Point3::new(0.0, 0.0, 0.0));
/// let v01 = builder::vertex(Point3::new(1.0, 0.0, 0.0));
/// let v02 = builder::vertex(Point3::new(2.0, 0.0, 0.0));
/// let v10 = builder::vertex(Point3::new(0.0, 1.0, 0.0));
/// let v11 = builder::vertex(Point3::new(1.0, 1.0, 0.0));
/// let v12 = builder::vertex(Point3::new(2.0, 1.0, 0.0));
/// let wire0 = wire![
///     builder::line(&v00, &v01),
///     builder::line(&v01, &v02),
/// ];
/// let wire1 = wire![
///     builder::line(&v10, &v11),
///     builder::line(&v11, &v12),
/// ];
///
/// let shell: Shell = builder::try_wire_homotopy(&wire0, &wire1)?;
/// assert_eq!(shell.len(), 2);
/// let boundary = shell.extract_boundaries();
/// assert_eq!(boundary.len(), 1);
/// assert_eq!(boundary[0].len(), 6);
/// # Ok(())
/// # }
/// ```
/// ```
/// # fn main() -> anyhow::Result<()> {
/// // a triangular tube
/// use monstertruck_modeling::*;
///
/// let v00 = builder::vertex(Point3::new(0.0, 0.0, 0.0));
/// let v01 = builder::vertex(Point3::new(1.0, 0.0, 0.0));
/// let v02 = builder::vertex(Point3::new(0.5, 0.5, 0.0));
/// let v10 = builder::vertex(Point3::new(0.0, 0.0, 1.0));
/// let v11 = builder::vertex(Point3::new(1.0, 0.0, 1.0));
/// let v12 = builder::vertex(Point3::new(0.5, 0.5, 1.0));
/// let wire0 = wire![
///     builder::line(&v00, &v01),
///     builder::line(&v01, &v02),
///     builder::line(&v02, &v00),
/// ];
/// let wire1 = wire![
///     builder::line(&v10, &v11),
///     builder::line(&v11, &v12),
///     builder::line(&v12, &v10),
/// ];
///
/// let shell: Shell = builder::try_wire_homotopy(&wire0, &wire1)?;
/// assert_eq!(shell.len(), 3);
/// let boundary = shell.extract_boundaries();
/// assert_eq!(boundary.len(), 2);
/// assert_eq!(boundary[0].len(), 3);
/// assert_eq!(boundary[1].len(), 3);
/// # Ok(())
/// # }
/// ```
/// # Failures
/// If the wires have different numbers of edges, then return `Error::NotSameNumberOfEdges`.
/// ```
/// use monstertruck_modeling::{*, errors::Error};
///
/// let v00 = builder::vertex(Point3::new(0.0, 0.0, 0.0));
/// let v01 = builder::vertex(Point3::new(1.0, 0.0, 0.0));
/// let v02 = builder::vertex(Point3::new(0.5, 0.5, 0.0));
/// let v10 = builder::vertex(Point3::new(0.0, 0.0, 1.0));
/// let v11 = builder::vertex(Point3::new(1.0, 0.0, 1.0));
/// let v12 = builder::vertex(Point3::new(0.5, 0.5, 1.0));
/// let wire0 = wire![
///     builder::line(&v00, &v01),
///     builder::line(&v01, &v02),
/// ];
/// let wire1 = wire![
///     builder::line(&v10, &v11),
///     builder::line(&v11, &v12),
///     builder::line(&v12, &v10),
/// ];
///
/// assert!(matches!(
///     builder::try_wire_homotopy::<Curve, Surface>(&wire0, &wire1),
///     Err(Error::NotSameNumberOfEdges),
/// ));
/// ```
pub fn try_wire_homotopy<C, S>(wire0: &Wire<C>, wire1: &Wire<C>) -> Result<Shell<C, S>>
where
    C: Invertible,
    Line<Point3>: ToSameGeometry<C>,
    HomotopySurface<C, C>: ToSameGeometry<S>, {
    if wire0.len() != wire1.len() {
        return Err(Error::NotSameNumberOfEdges);
    }
    let mut vemap = monstertruck_core::entry_map::FxEntryMap::new(
        |(v0, v1): (&Vertex, &Vertex)| (v0.id(), v1.id()),
        |(v0, v1)| line(v0, v1),
    );
    let shell = wire0
        .edge_iter()
        .zip(wire1.edge_iter())
        .map(|(edge0, edge1)| {
            let (v0, v1) = (edge0.front(), edge1.front());
            let edge2 = vemap.entry_or_insert((v0, v1)).inverse();
            let (v0, v1) = (edge0.back(), edge1.back());
            let edge3 = vemap.entry_or_insert((v0, v1)).clone();
            let wire = wire![edge0.clone(), edge3, edge1.inverse(), edge2];
            let curve0 = edge0.oriented_curve();
            let curve1 = edge1.oriented_curve();
            let homotopy = HomotopySurface::new(curve0, curve1);
            Face::new_unchecked(vec![wire], homotopy.to_same_geometry())
        })
        .collect();
    Ok(shell)
}

/// Skins (lofts) a sequence of wires into a shell by connecting adjacent pairs
/// with homotopy faces.
///
/// Each adjacent pair of wires must have the same number of edges. The result
/// is a shell with `(N-1) * edges_per_wire` faces, where N is the number of
/// input wires. Shared edges between adjacent strips are automatically reused.
///
/// # Errors
///
/// - [`Error::NotSameNumberOfEdges`] if any adjacent pair has different edge counts.
/// - Returns an error if fewer than 2 wires are provided.
///
/// # Examples
///
/// ```
/// use monstertruck_modeling::*;
///
/// // Three parallel line segments skinned into two quad faces.
/// let v0 = builder::vertex(Point3::new(0.0, 0.0, 0.0));
/// let v1 = builder::vertex(Point3::new(1.0, 0.0, 0.0));
/// let v2 = builder::vertex(Point3::new(0.0, 1.0, 0.0));
/// let v3 = builder::vertex(Point3::new(1.0, 1.0, 0.0));
/// let v4 = builder::vertex(Point3::new(0.0, 2.0, 0.0));
/// let v5 = builder::vertex(Point3::new(1.0, 2.0, 0.0));
/// let w0: Wire = vec![builder::line(&v0, &v1)].into();
/// let w1: Wire = vec![builder::line(&v2, &v3)].into();
/// let w2: Wire = vec![builder::line(&v4, &v5)].into();
/// let shell: Shell = builder::try_skin_wires(&[w0, w1, w2]).unwrap();
/// assert_eq!(shell.len(), 2);
/// ```
pub fn try_skin_wires<C, S>(wires: &[Wire<C>]) -> Result<Shell<C, S>>
where
    C: Invertible,
    Line<Point3>: ToSameGeometry<C>,
    HomotopySurface<C, C>: ToSameGeometry<S>, {
    if wires.len() < 2 {
        return Err(Error::NotSameNumberOfEdges);
    }
    // Vertex-edge map shared across all strips so adjacent strips reuse edges.
    let mut vemap = monstertruck_core::entry_map::FxEntryMap::new(
        |(v0, v1): (&Vertex, &Vertex)| (v0.id(), v1.id()),
        |(v0, v1)| line(v0, v1),
    );
    let mut shell = Shell::new();
    for pair in wires.windows(2) {
        let (w0, w1) = (&pair[0], &pair[1]);
        if w0.len() != w1.len() {
            return Err(Error::NotSameNumberOfEdges);
        }
        let strip: Shell<_, _> = w0
            .edge_iter()
            .zip(w1.edge_iter())
            .map(|(edge0, edge1)| {
                let (va, vb) = (edge0.front(), edge1.front());
                let edge2 = vemap.entry_or_insert((va, vb)).inverse();
                let (va, vb) = (edge0.back(), edge1.back());
                let edge3 = vemap.entry_or_insert((va, vb)).clone();
                let wire = wire![edge0.clone(), edge3, edge1.inverse(), edge2];
                let curve0 = edge0.oriented_curve();
                let curve1 = edge1.oriented_curve();
                let homotopy = HomotopySurface::new(curve0, curve1);
                Face::new_unchecked(vec![wire], homotopy.to_same_geometry())
            })
            .collect();
        shell.extend(strip);
    }
    Ok(shell)
}

/// Try attatiching a plane whose boundary is `wire`.
/// # Examples
/// ```
/// # fn main() -> anyhow::Result<()> {
/// use monstertruck_modeling::*;
/// use monstertruck_modeling::builder::SweepAngle;
///
/// // make a disk by attaching a plane into circle
/// let vertex: Vertex = builder::vertex(Point3::new(1.0, 0.0, 0.0));
/// let circle: Wire = builder::revolve(&vertex, Point3::origin(), Vector3::unit_y(), SweepAngle::Closed, 2);
/// let disk: Face = builder::try_attach_plane(vec![circle])?;
/// # let surface = disk.oriented_surface();
/// # let normal = surface.normal(0.5, 0.5);
/// # assert!(normal.near(&Vector3::unit_y()));
/// # Ok(())
/// # }
/// ```
/// # Failures
/// If `wires`` are not in one plane, then return `Error::WireNotInOnePlane`.
/// ```
/// use monstertruck_modeling::{*, errors::Error};
/// let v0 = builder::vertex(Point3::new(0.0, 0.0, 0.0));
/// let v1 = builder::vertex(Point3::new(1.0, 0.0, 0.0));
/// let v2 = builder::vertex(Point3::new(0.0, 1.0, 0.0));
/// let v3 = builder::vertex(Point3::new(0.0, 0.0, 1.0));
/// let wire: Wire = vec![
///     builder::line(&v0, &v1),
///     builder::line(&v1, &v2),
/// ]
/// .into();
/// let mut wires = vec![wire];
/// // failed to attach plane, because wire is not closed.
/// assert_eq!(
///     builder::try_attach_plane::<_, Surface>(wires.clone()).unwrap_err(),
///     Error::FromTopology(monstertruck_topology::errors::Error::NotClosedWire),
/// );
///
/// wires[0].push_back(builder::line(&v2, &v3));
/// wires[0].push_back(builder::line(&v3, &v0));
/// // failed to attach plane, because wire is not in the plane.
/// assert_eq!(
///     builder::try_attach_plane::<_, Surface>(wires.clone()).unwrap_err(),
///     Error::WireNotInOnePlane,
/// );
///
/// wires[0].pop_back();
/// wires[0].pop_back();
/// wires[0].push_back(builder::line(&v2, &v0));
/// // success in attaching plane!
/// assert!(builder::try_attach_plane::<_, Surface>(wires).is_ok());
/// ```
pub fn try_attach_plane<C, S>(wires: impl Into<Vec<Wire<C>>>) -> Result<Face<C, S>>
where
    C: ParametricCurve3D + BoundedCurve,
    Plane: IncludeCurve<C> + ToSameGeometry<S>, {
    let wires = wires.into();
    let _ = Face::try_new(wires.clone(), ())?;
    let pts = wires
        .iter()
        .map(|wire| {
            wire.edge_iter()
                .flat_map(|edge| {
                    let p0 = edge.front().point();
                    let curve = edge.curve();
                    let (t0, t1) = curve.range_tuple();
                    let p1 = curve.subs((t0 + t1) / 2.0);
                    [p0, p1]
                })
                .collect()
        })
        .collect::<Vec<_>>();

    let plane = match geom_impls::attach_plane(pts) {
        Some(got) => got,
        None => return Err(Error::WireNotInOnePlane),
    };
    Ok(Face::new_unchecked(wires, plane.to_same_geometry()))
}

/// Returns another topology whose points, curves, and surfaces are cloned.
/// # Examples
/// ```
/// use monstertruck_modeling::*;
/// let v = builder::vertex(Point3::origin());
/// let v0 = builder::clone(&v);
/// assert_eq!(v0.point(), Point3::origin());
/// assert_ne!(v0.id(), v.id());
/// ```
#[inline(always)]
pub fn clone<T: Mapped<()>>(elem: &T) -> T { elem.mapped(()) }

/// Returns a transformed vertex, edge, wire, face, shell or solid.
#[inline(always)]
pub fn transformed<T: Mapped<Matrix4>>(elem: &T, mat: Matrix4) -> T { elem.mapped(mat) }

/// Returns a translated vertex, edge, wire, face, shell or solid.
#[inline(always)]
pub fn translated<T: Mapped<Matrix4>>(elem: &T, vector: Vector3) -> T {
    transformed(elem, Matrix4::from_translation(vector))
}

/// Returns a rotated vertex, edge, wire, face, shell or solid.
pub fn rotated<T: Mapped<Matrix4>>(elem: &T, origin: Point3, axis: Vector3, angle: Rad<f64>) -> T {
    let mat0 = Matrix4::from_translation(-origin.to_vec());
    let mat1 = Matrix4::from_axis_angle(axis, angle);
    let mat2 = Matrix4::from_translation(origin.to_vec());
    transformed(elem, mat2 * mat1 * mat0)
}

/// Returns a scaled vertex, edge, wire, face, shell or solid.
pub fn scaled<T: Mapped<Matrix4>>(elem: &T, origin: Point3, scalars: Vector3) -> T {
    let mat0 = Matrix4::from_translation(-origin.to_vec());
    let mat1 = Matrix4::from_nonuniform_scale(scalars[0], scalars[1], scalars[2]);
    let mat2 = Matrix4::from_translation(origin.to_vec());
    transformed(elem, mat2 * mat1 * mat0)
}

/// Sweeps a vertex, an edge, a wire, a face, or a shell by a vector.
///
/// # Examples
/// ```
/// use monstertruck_modeling::*;
/// let vertex = builder::vertex(Point3::new(0.0, 0.0, 0.0));
/// let line = builder::extrude(&vertex, Vector3::unit_x());
/// let square = builder::extrude(&line, Vector3::unit_y());
/// let cube: Solid = builder::extrude(&square, Vector3::unit_z());
/// #
/// # let b_shell = &cube.boundaries()[0];
/// # assert_eq!(b_shell.len(), 6); // This solid is a cube!
/// # assert!(cube.is_geometric_consistent());
/// #
/// # let b_loop = &b_shell[0].boundaries()[0];
/// # let mut loop_iter = b_loop.vertex_iter();
/// # assert_eq!(loop_iter.next().unwrap().point(), Point3::new(0.0, 0.0, 0.0));
/// # assert_eq!(loop_iter.next().unwrap().point(), Point3::new(0.0, 1.0, 0.0));
/// # assert_eq!(loop_iter.next().unwrap().point(), Point3::new(1.0, 1.0, 0.0));
/// # assert_eq!(loop_iter.next().unwrap().point(), Point3::new(1.0, 0.0, 0.0));
/// # assert_eq!(loop_iter.next(), None);
/// #
/// # let b_loop = &b_shell[3].boundaries()[0];
/// # let mut loop_iter = b_loop.vertex_iter();
/// # assert_eq!(loop_iter.next().unwrap().point(), Point3::new(1.0, 1.0, 0.0));
/// # assert_eq!(loop_iter.next().unwrap().point(), Point3::new(0.0, 1.0, 0.0));
/// # assert_eq!(loop_iter.next().unwrap().point(), Point3::new(0.0, 1.0, 1.0));
/// # assert_eq!(loop_iter.next().unwrap().point(), Point3::new(1.0, 1.0, 1.0));
/// # assert_eq!(loop_iter.next(), None);
/// #
/// # let b_loop = &b_shell[5].boundaries()[0];
/// # let mut loop_iter = b_loop.vertex_iter();
/// # assert_eq!(loop_iter.next().unwrap().point(), Point3::new(0.0, 0.0, 1.0));
/// # assert_eq!(loop_iter.next().unwrap().point(), Point3::new(1.0, 0.0, 1.0));
/// # assert_eq!(loop_iter.next().unwrap().point(), Point3::new(1.0, 1.0, 1.0));
/// # assert_eq!(loop_iter.next().unwrap().point(), Point3::new(0.0, 1.0, 1.0));
/// # assert_eq!(loop_iter.next(), None);
/// ```
///
/// # Requirement
/// In order to apply this method to `Vertex<Point3>`, ..., `Shell<Point3, C, S>`, the following constraints must be satisfied.
/// ```ignore
/// C: Transformed<Matrix4>,
/// S: Transformed<Matrix4>,
/// Line<Point3>: ToSameGeometry<C>,
/// ExtrusionSurface<C, Vector3>: ToSameGeometry<S>
/// ```
pub fn extrude<T, Swept>(elem: &T, vector: Vector3) -> Swept
where T: Sweep<Matrix4, LineConnector, ExtrudeConnector, Swept> {
    let trsl = Matrix4::from_translation(vector);
    elem.sweep(trsl, LineConnector, ExtrudeConnector { vector })
}

/// Sweeps a vertex, an edge, a wire, a face, or a shell by the rotation.
/// # Details
/// If the absolute value of `angle` is more than 2π rad, then the result is closed shape.
/// For example, the result of sweeping a disk is a bent cylinder if `angle` is less than 2π rad
/// and a solid torus if `angle` is no less than 2π rad.
/// # Remarks
/// `axis` must be normalized. If not, panics occurs in debug mode.
/// # Panics
/// ALways `division > 0` must hold. Moreover, `division >= 2` must hold if `angle` is no less than 2π.
/// # Examples
/// ```
/// // Torus
/// use monstertruck_modeling::*;
/// use monstertruck_modeling::builder::SweepAngle;
///
/// let v = builder::vertex(Point3::new(3.0, 0.0, 0.0));
/// let circle = builder::revolve(&v, Point3::new(2.0, 0.0, 0.0), Vector3::unit_z(), SweepAngle::Closed, 2);
/// let torus = builder::revolve(&circle, Point3::origin(), Vector3::unit_y(), SweepAngle::Closed, 2);
/// let solid: Solid = Solid::new(vec![torus]);
/// #
/// # assert!(solid.is_geometric_consistent());
/// # const N: usize = 100;
/// # let shell = &solid.boundaries()[0];
/// # for face in shell.iter() {
/// #   let surface = face.surface();
/// #   for i in 0..=N {
/// #       for j in 0..=N {
/// #           let u = i as f64 / N as f64;
/// #           let v = j as f64 / N as f64;
/// #           let pt = surface.subs(u, v);
/// #
/// #           // this surface is a part of torus.
/// #           let tmp = f64::sqrt(pt[0] * pt[0] + pt[2] * pt[2]) - 2.0;
/// #           let res = tmp * tmp + pt[1] * pt[1];
/// #           assert!(Tolerance::near(&res, &1.0));
/// #       }
/// #    }
/// # }
/// ```
/// ```
/// // Modeling a pipe.
/// use monstertruck_modeling::*;
/// use monstertruck_modeling::builder::SweepAngle;
/// const PI: Rad<f64> = Rad(std::f64::consts::PI);
///
/// // Creates the base circle
/// let v: Vertex = builder::vertex(Point3::new(1.0, 0.0, 4.0));
/// let circle: Wire = builder::revolve(&v, Point3::new(2.0, 0.0, 4.0), -Vector3::unit_z(), SweepAngle::Closed, 2);
///
/// // the result shell of the pipe.
/// let mut pipe: Shell = Shell::new();
///
/// // Draw the first line pipe
/// let mut first_line_part: Shell = builder::extrude(&circle, Vector3::new(0.0, 0.0, -4.0));
/// pipe.append(&mut first_line_part);
///
/// // Get the new wire
/// let boundaries: Vec<Wire> = pipe.extract_boundaries();
/// let another_circle: Wire = boundaries.into_iter().find(|wire| wire != &circle).unwrap().inverse();
///
/// // Draw the bent part
/// let mut bend_part: Shell = builder::revolve(
///     &another_circle,
///     Point3::origin(),
///     Vector3::unit_y(),
///     SweepAngle::Partial(PI / 2.0),
///     2,
/// );
/// # let surface = bend_part[0].surface();
/// pipe.append(&mut bend_part);
///
/// // Get the new wire
/// let boundaries: Vec<Wire> = pipe.extract_boundaries();
/// let another_circle: Wire = boundaries.into_iter().find(|wire| wire != &circle).unwrap().inverse();
///
/// // Draw the second line pipe
/// let mut second_line_part: Shell = builder::extrude(&another_circle, Vector3::new(-4.0, 0.0, 0.0));
/// pipe.append(&mut second_line_part);
///
/// assert_eq!(pipe.shell_condition(), ShellCondition::Oriented);
/// # assert!(pipe.is_geometric_consistent());
/// # const N: usize = 100;
/// # for i in 0..=N {
/// #    for j in 0..=N {
/// #        let u = i as f64 / N as f64;
/// #        let v = j as f64 / N as f64;
/// #        let pt = surface.subs(u, v);
/// #
/// #        // the y coordinate is positive.
/// #        //assert!(pt[1] >= 0.0);
/// #
/// #        // this surface is a part of torus.
/// #        let tmp = f64::sqrt(pt[0] * pt[0] + pt[2] * pt[2]) - 2.0;
/// #        let res = tmp * tmp + pt[1] * pt[1];
/// #        assert!(Tolerance::near(&res, &1.0));
/// #    }
/// # }
/// ```
///
/// # Requirement
/// In order to apply this method to `Vertex<Point3>`, ..., `Shell<Point3, C, S>`, the following constraints must be satisfied.
/// ```ignore
/// C: Transformed<Matrix4>,
/// S: Transformed<Matrix4>,
/// Processor<TrimmedCurve<UnitCircle<Point3>>, Matrix4>: ToSameGeometry<C>,
/// RevolutionSurface<C>: ToSameGeometry<S>,
/// ```
pub fn revolve<T, Swept>(
    elem: &T,
    origin: Point3,
    axis: Vector3,
    sweep: SweepAngle,
    division: usize,
) -> Swept
where
    T: ClosedSweep<Matrix4, ArcConnector, RevoluteConnector, Swept>,
{
    debug_assert!(axis.magnitude().near(&1.0));
    match sweep {
        SweepAngle::Closed => {
            assert!(
                division >= 2,
                "division must be 2 or greater for closed revolve"
            );
            whole_revolve(elem, origin, axis, division)
        }
        SweepAngle::Partial(angle) => {
            assert!(division >= 1, "division must be 1 or greater");
            let sign = f64::signum(angle.0);
            partial_revolve(elem, origin, sign * axis, angle * sign, division)
        }
    }
}

fn partial_revolve<T: MultiSweep<Matrix4, ArcConnector, RevoluteConnector, Swept>, Swept>(
    elem: &T,
    origin: Point3,
    axis: Vector3,
    angle: Rad<f64>,
    division: usize,
) -> Swept {
    let mat0 = Matrix4::from_translation(-origin.to_vec());
    let mat1 = Matrix4::from_axis_angle(axis, angle / division as f64);
    let mat2 = Matrix4::from_translation(origin.to_vec());
    let trsl = mat2 * mat1 * mat0;
    elem.multi_sweep(
        trsl,
        ArcConnector {
            origin,
            axis,
            angle: angle / division as f64,
        },
        RevoluteConnector { origin, axis },
        division,
    )
}

fn whole_revolve<T: ClosedSweep<Matrix4, ArcConnector, RevoluteConnector, Swept>, Swept>(
    elem: &T,
    origin: Point3,
    axis: Vector3,
    division: usize,
) -> Swept {
    let mat0 = Matrix4::from_translation(-origin.to_vec());
    let mat1 = Matrix4::from_axis_angle(axis, PI * 2.0 / division as f64);
    let mat2 = Matrix4::from_translation(origin.to_vec());
    let trsl = mat2 * mat1 * mat0;
    elem.closed_sweep(
        trsl,
        ArcConnector {
            origin,
            axis,
            angle: PI * 2.0 / division as f64,
        },
        RevoluteConnector { origin, axis },
        division,
    )
}

/// Revolves a wire around an axis, automatically collapsing degenerate edges
/// where wire endpoints lie on the rotation axis.
///
/// Unlike [`revolve`] (which produces zero-length edges at on-axis points),
/// this function detects vertices on the axis and produces clean 3-sided
/// faces instead of degenerate 4-sided ones.
///
/// # Examples
///
/// ```
/// use monstertruck_modeling::*;
/// use std::f64::consts::PI;
///
/// let v0 = builder::vertex(Point3::new(0.0, 1.0, 0.0));
/// let v1 = builder::vertex(Point3::new(0.0, 0.0, 1.0));
/// let v2 = builder::vertex(Point3::new(0.0, 0.0, 0.0));
/// let wire: Wire = vec![
///     builder::line(&v0, &v1),
///     builder::line(&v1, &v2),
/// ].into();
/// let shell = builder::revolve_wire(
///     &wire,
///     Point3::origin(),
///     Vector3::unit_y(),
///     builder::SweepAngle::Closed,
///     4,
/// );
/// // Degenerate edges are removed -- faces have 3 edges, not 4.
/// assert_eq!(shell[0].boundaries()[0].len(), 3);
/// // The result is a valid closed shell.
/// Solid::new(vec![shell]);
/// ```
pub fn revolve_wire<C, S>(
    wire: &Wire<C>,
    origin: Point3,
    axis: Vector3,
    sweep: SweepAngle,
    division: usize,
) -> Shell<C, S>
where
    C: ParametricCurve3D + BoundedCurve + Cut + Invertible + Transformed<Matrix4>,
    S: Invertible,
    Processor<TrimmedCurve<UnitCircle<Point3>>, Matrix4>: ToSameGeometry<C>,
    RevolutionSurface<C>: ToSameGeometry<S>,
{
    let closed = matches!(sweep, SweepAngle::Closed);
    let mut wire = wire.clone();
    if wire.is_empty() {
        return Shell::new();
    }

    let on_axis = |pt: Point3| (pt - origin).cross(axis).so_small();
    let front_on_axis = on_axis(wire.front_vertex().unwrap().point());
    let back_on_axis = on_axis(wire.back_vertex().unwrap().point());

    // If the wire is a single edge with the back vertex on-axis,
    // split it at the midpoint so the revolve can produce proper faces.
    if wire.len() == 1 && back_on_axis {
        let edge = wire.pop_back().unwrap();
        let v0 = edge.front().clone();
        let v2 = edge.back().clone();
        let mut curve = edge.curve();
        let (t0, t1) = curve.range_tuple();
        let t = (t0 + t1) * 0.5;
        let v1 = Vertex::new(curve.subs(t));
        let curve1 = curve.cut(t);
        // Spec 012 U4: `v1` is a freshly allocated `Vertex`, and `Vertex`
        // equality is `Arc` pointer identity, so neither of these two edges can
        // be degenerate. Vacuous check, infallible caller: stated, not
        // profile-switched.
        wire.push_back(Edge::new_unchecked(&v0, &v1, curve));
        wire.push_back(Edge::new_unchecked(&v1, &v2, curve1));
    }

    let mut shell = revolve(&wire, origin, axis, sweep, division);

    // Collapse degenerate edges at the front of the wire (on-axis).
    if front_on_axis {
        let mut edge = shell[0].boundaries()[0][0].clone();
        for i in 0..shell.len() / wire.len() {
            let idx = i * wire.len();
            let face = shell[idx].clone();
            let surface = face.oriented_surface();
            let old_wire = face.into_boundaries().pop().unwrap();
            let mut new_wire = Wire::new();
            new_wire.push_back(edge.clone());
            new_wire.push_back(old_wire[1].clone());
            let new_edge = if closed && i + 1 == shell.len() / wire.len() {
                shell[0].boundaries()[0][0].inverse()
            } else {
                let curve = old_wire[2].oriented_curve();
                // Spec 012 U4: `revolve_wire` returns a bare `Shell`, so there
                // is no channel. Unlike the two above this is NOT provably
                // vacuous -- both vertices come out of the revolve's own shell.
                // Stated as unchecked rather than profile-switched (abort in
                // debug, this exact call in release); the check it drops is on
                // the builder's own output and has never fired in-gate.
                Edge::new_unchecked(old_wire[2].front(), new_wire[0].front(), curve)
            };
            new_wire.push_back(new_edge.clone());
            shell[idx] = Face::new_unchecked(vec![new_wire], surface);
            edge = new_edge.inverse();
        }
    }

    // Collapse degenerate edges at the back of the wire (on-axis).
    if back_on_axis {
        let mut edge = shell[wire.len() - 1].boundaries()[0][0].clone();
        for i in 0..shell.len() / wire.len() {
            let idx = (i + 1) * wire.len() - 1;
            let face = shell[idx].clone();
            let surface = face.oriented_surface();
            let old_wire = face.into_boundaries().pop().unwrap();
            let mut new_wire = Wire::new();
            new_wire.push_back(edge.clone());
            let new_edge = if closed && i + 1 == shell.len() / wire.len() {
                shell[wire.len() - 1].boundaries()[0][0].inverse()
            } else {
                let curve = old_wire[2].oriented_curve();
                // Spec 012 U4, mirror of the front-collapse case above.
                Edge::new_unchecked(new_wire[0].back(), old_wire[2].back(), curve)
            };
            new_wire.push_back(new_edge.clone());
            new_wire.push_back(old_wire[3].clone());
            shell[idx] = Face::new_unchecked(vec![new_wire], surface);
            edge = new_edge.inverse();
        }
    }
    shell
}

/// Use [`revolve_wire`] instead, which takes an explicit `origin` parameter
/// and handles on-axis degenerate edges automatically.
#[deprecated(note = "Use revolve_wire instead, which takes an explicit origin parameter.")]
pub fn cone<C, S>(
    wire: &Wire<C>,
    axis: Vector3,
    sweep: SweepAngle,
    division: usize,
) -> Shell<C, S>
where
    C: ParametricCurve3D + BoundedCurve + Cut + Invertible + Transformed<Matrix4>,
    S: Invertible,
    Processor<TrimmedCurve<UnitCircle<Point3>>, Matrix4>: ToSameGeometry<C>,
    RevolutionSurface<C>: ToSameGeometry<S>,
{
    let origin = wire.front_vertex().map_or(Point3::origin(), |v| v.point());
    revolve_wire(wire, origin, axis, sweep, division)
}

#[cfg(test)]
mod partial_torus;
