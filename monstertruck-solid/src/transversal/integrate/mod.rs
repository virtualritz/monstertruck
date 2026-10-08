// The classic backend is the only `super` item this module dispatches to; the
// public trait bounds (`SnapCurveEndpoints`, etc.) resolve through the prelude
// globs below, so a no-backend (`--no-default-features`) build needs no `super`
// glob.
#[cfg(feature = "marching-ssi")]
use super::classic;
use monstertruck_geometry::prelude::*;
use monstertruck_meshing::tessellation::{ExactTrimBoundary2D, Parallelizable};
use monstertruck_topology::{errors::Error as TopologyError, shell::ShellCondition, *};
use std::iter::{once, repeat};
use thiserror::Error;

/// Only solids consisting of faces whose surface is implemented this trait can be used for set operations.
pub trait ShapeOpsSurface:
    ParametricSurface3D
    + ParameterDivision2D
    + SearchParameter<SurfaceParameter, Point = Point3>
    + SearchNearestParameter<SurfaceParameter, Point = Point3>
    + SupportsExactPatchDomains
    + TryIntoAnalyticSurfaceKind
    + TryIntoBsplineSurface
    + TryIntoHomogeneousBsplineSurface
    + Clone
    + Invertible
    + Send
    + Sync {
}
impl<S> ShapeOpsSurface for S where S: ParametricSurface3D
        + ParameterDivision2D
        + SearchParameter<SurfaceParameter, Point = Point3>
        + SearchNearestParameter<SurfaceParameter, Point = Point3>
        + SupportsExactPatchDomains
        + TryIntoAnalyticSurfaceKind
        + TryIntoBsplineSurface
        + TryIntoHomogeneousBsplineSurface
        + Clone
        + Invertible
        + Send
        + Sync
{
}

/// Only solids consisting of edges whose curve is implemented this trait can be used for set operations.
pub trait ShapeOpsCurve<S: ShapeOpsSurface>:
    ParametricCurve3D
    + ParameterDivision1D<Point = Point3>
    + ParameterBoundary2D<S>
    + Cut
    + Clone
    + TryFrom<ParameterCurve<Line<Point2>, S>>
    + Invertible
    + From<
        SurfaceCurve<
            BsplineCurve<Point3>,
            S,
            S,
            ParameterCurve<BoundaryCurve2D, S>,
            ParameterCurve<BoundaryCurve2D, S>,
        >,
    > + From<
        SurfaceCurve<
            NurbsCurve<Vector4>,
            S,
            S,
            ParameterCurve<BoundaryCurve2D, S>,
            ParameterCurve<BoundaryCurve2D, S>,
        >,
    > + SearchParameter<CurveParameter, Point = Point3>
    + SearchNearestParameter<CurveParameter, Point = Point3>
    + SnapCurveEndpoints
    + Send
    + Sync {
}
impl<C, S: ShapeOpsSurface> ShapeOpsCurve<S> for C where C: ParametricCurve3D
        + ParameterDivision1D<Point = Point3>
        + ParameterBoundary2D<S>
        + Cut
        + Clone
        + TryFrom<ParameterCurve<Line<Point2>, S>>
        + Invertible
        + From<
            SurfaceCurve<
                BsplineCurve<Point3>,
                S,
                S,
                ParameterCurve<BoundaryCurve2D, S>,
                ParameterCurve<BoundaryCurve2D, S>,
            >,
        > + From<
            SurfaceCurve<
                NurbsCurve<Vector4>,
                S,
                S,
                ParameterCurve<BoundaryCurve2D, S>,
                ParameterCurve<BoundaryCurve2D, S>,
            >,
        > + SearchParameter<CurveParameter, Point = Point3>
        + SearchNearestParameter<CurveParameter, Point = Point3>
        + SnapCurveEndpoints
        + Send
        + Sync
{
}

/// Internal extension for boolean stages that preserve exact face-local trims.
pub trait TrimAwareShapeOpsCurve<S: ShapeOpsSurface>:
    ShapeOpsCurve<S> + ExactParameterBoundary2D<S> {
}

impl<C, S: ShapeOpsSurface> TrimAwareShapeOpsCurve<S> for C where C: ShapeOpsCurve<S> + ExactParameterBoundary2D<S> {}

/// Internal exact-trim boolean lane whose face-local trims can be split.
pub trait CuttableTrimAwareShapeOpsCurve<S: ShapeOpsSurface>: TrimAwareShapeOpsCurve<S>
where <Self as ExactParameterBoundary2D<S>>::BoundaryCurve: ParametricCurve3D<Point = Point3>
        + BoundedCurve<Point = Point3>
        + SearchNearestParameter<CurveParameter, Point = Point3>
        + Cut
        + Clone
        + Invertible {
}

impl<C, S: ShapeOpsSurface> CuttableTrimAwareShapeOpsCurve<S> for C
where
    C: TrimAwareShapeOpsCurve<S>,
    <C as ExactParameterBoundary2D<S>>::BoundaryCurve: ParametricCurve3D<Point = Point3>
        + BoundedCurve<Point = Point3>
        + SearchNearestParameter<CurveParameter, Point = Point3>
        + Cut
        + Clone
        + Invertible,
{
}

/// Errors for boolean shape operations.
#[derive(Debug, Error)]
pub enum ShapeOpsError {
    /// `tol` was not positive enough for robust meshing and projection.
    #[error("`tol` must be at least `TOLERANCE`.")]
    InvalidTolerance,
    /// `world_to_clip` could not be inverted.
    #[error("`world_to_clip` must be invertible for `clip_half_space_z`.")]
    NonInvertibleClipTransform,
    /// No boolean backend is compiled in. Reachable only under
    /// `--no-default-features` (the default `marching-ssi` backend disabled and
    /// no upgrade backend linked). Enable `marching-ssi` (the published default)
    /// or link an external SSI boolean backend.
    #[error(
        "no boolean backend available: enable `marching-ssi` (default) or link an external SSI backend."
    )]
    NoBackend,
    /// Face division failed for one shell.
    #[error("failed to divide faces for shell {shell_index}.")]
    DivideFacesFailed {
        /// 0 for the first shell, 1 for the second shell.
        shell_index: usize,
    },
    /// Unknown face classification failed for one shell.
    #[error("failed to classify unknown faces for shell {shell_index}.")]
    UnknownClassificationFailed {
        /// 0 for the first shell, 1 for the second shell.
        shell_index: usize,
    },
    /// Converting temporary intersection curves back to target curves failed.
    #[error("failed to convert temporary shell for `{operation}`.")]
    AltShellConversionFailed {
        /// `and` or `or`.
        operation: &'static str,
    },
    /// The generated shell failed manifold checks before solid construction.
    #[error(transparent)]
    InvalidOutputShellCondition(Box<InvalidOutputShellConditionData>),
    /// The output has no boundary shells.
    #[error("invalid output shell for `{operation}`: no boundary shells.")]
    EmptyOutputShell {
        /// Boolean operation name.
        operation: &'static str,
    },
    /// The generated shell is topologically invalid.
    #[error("invalid output shell for `{operation}`: {source}.")]
    InvalidOutputShell {
        /// Boolean operation name.
        operation: &'static str,
        /// Topology validation error.
        #[source]
        source: TopologyError,
    },
}

/// Diagnostic data for invalid output shell conditions.
#[derive(Debug, Error)]
#[error(
    "invalid output shell for `{operation}` at index {shell_index}: empty={empty}, connected={connected}, condition={condition:?}, boundary_loops={boundary_loops}, first_boundary_len={first_boundary_len:?}, first_boundary_front={first_boundary_front:?}, first_boundary_back={first_boundary_back:?}, singular_vertices={singular_vertices}, first_singular={first_singular:?}."
)]
pub struct InvalidOutputShellConditionData {
    /// Boolean operation name.
    pub operation: &'static str,
    /// Boundary shell index.
    pub shell_index: usize,
    /// Whether shell has no faces.
    pub empty: bool,
    /// Whether shell is topologically connected.
    pub connected: bool,
    /// Evaluated shell condition.
    pub condition: ShellCondition,
    /// Count of extracted open boundary wires.
    pub boundary_loops: usize,
    /// Number of edges in first open boundary wire.
    pub first_boundary_len: Option<usize>,
    /// Front point of first open boundary wire.
    pub first_boundary_front: Option<Point3>,
    /// Back point of first open boundary wire.
    pub first_boundary_back: Option<Point3>,
    /// Number of singular vertices.
    pub singular_vertices: usize,
    /// First singular vertex point if present.
    pub first_singular: Option<Point3>,
}

type ShapeOpsResult<T> = std::result::Result<T, ShapeOpsError>;

const HALF_SPACE_CLIP_OPERATION: &str = "clip_half_space_z";

/// Orientation hints for the combined input shells of a boolean operation.
///
/// `monstertruck-solid` normally derives shell orientation by tessellating the
/// full input shells and measuring signed volume. When the caller already knows
/// whether either combined shell is inverted, passing these hints avoids that
/// extra global polyline conversion work.
#[derive(Clone, Copy, Debug, Default)]
pub struct ShellOrientationHints {
    /// Whether the first combined shell is inverted.
    pub first_inverted: bool,
    /// Whether the second combined shell is inverted.
    pub second_inverted: bool,
}

/// AND operation between two solids.
pub fn and<C: CuttableTrimAwareShapeOpsCurve<S>, S: ShapeOpsSurface>(
    solid0: &Solid<Point3, C, S>,
    solid1: &Solid<Point3, C, S>,
    tol: f64,
) -> ShapeOpsResult<Solid<Point3, C, S>>
where
    Plane: IncludeCurve<C> + ToSameGeometry<S>,
    Line<Point3>: ToSameGeometry<C>,
    <C as ExactParameterBoundary2D<S>>::BoundaryCurve: BoundedCurve
        + BoundaryCurveFromSamples<S>
        + Cut
        + Clone
        + Invertible
        + ExactTrimBoundary2D
        + Parallelizable,
{
    #[cfg(feature = "marching-ssi")]
    {
        classic::and(solid0, solid1, tol)
    }
    #[cfg(not(feature = "marching-ssi"))]
    {
        let _ = (solid0, solid1, tol);
        Err(ShapeOpsError::NoBackend)
    }
}

/// AND operation between two solids with known combined-shell orientation.
///
/// This is a performance-oriented variant of [`and`] for callers that already
/// know whether either input solid's combined shell is inverted. It avoids the
/// fallback full-shell tessellation used only to recover shell orientation.
///
/// Passing wrong hints can misclassify faces and produce invalid output.
pub fn and_with_orientation_hints<C: CuttableTrimAwareShapeOpsCurve<S>, S: ShapeOpsSurface>(
    solid0: &Solid<Point3, C, S>,
    solid1: &Solid<Point3, C, S>,
    orientation_hints: ShellOrientationHints,
    tol: f64,
) -> ShapeOpsResult<Solid<Point3, C, S>>
where
    Plane: IncludeCurve<C> + ToSameGeometry<S>,
    Line<Point3>: ToSameGeometry<C>,
    <C as ExactParameterBoundary2D<S>>::BoundaryCurve: BoundedCurve
        + BoundaryCurveFromSamples<S>
        + Cut
        + Clone
        + Invertible
        + ExactTrimBoundary2D
        + Parallelizable,
{
    #[cfg(feature = "marching-ssi")]
    {
        // The classic backend does not consume orientation hints; it derives
        // shell orientation from tessellated signed volume internally.
        let _ = orientation_hints;
        classic::and(solid0, solid1, tol)
    }
    #[cfg(not(feature = "marching-ssi"))]
    {
        let _ = (solid0, solid1, orientation_hints, tol);
        Err(ShapeOpsError::NoBackend)
    }
}

fn transformed_solid<C, S>(solid: &Solid<Point3, C, S>, transform: Matrix4) -> Solid<Point3, C, S>
where
    C: Transformed<Matrix4>,
    S: Transformed<Matrix4>, {
    solid.mapped(
        |point| transform.transform_point(*point),
        |curve| curve.transformed(transform),
        |surface| surface.transformed(transform),
    )
}

fn solid_bounds<C, S>(solid: &Solid<Point3, C, S>) -> BoundingBox<Point3> {
    solid.vertex_iter().map(|vertex| vertex.point()).collect()
}

fn half_space_contains_bounds(
    bounds: BoundingBox<Point3>,
    keep_positive_z: bool,
    tol: f64,
) -> bool {
    if keep_positive_z {
        bounds.min().z >= -tol
    } else {
        bounds.max().z <= tol
    }
}

fn half_space_discards_bounds(
    bounds: BoundingBox<Point3>,
    keep_positive_z: bool,
    tol: f64,
) -> bool {
    if keep_positive_z {
        bounds.max().z < -tol
    } else {
        bounds.min().z > tol
    }
}

fn half_space_clip_margin(bounds: BoundingBox<Point3>, tol: f64) -> f64 {
    let diameter = bounds.diameter().max(1.0);
    (diameter * 1.0e-3).max(10.0 * tol).max(100.0 * TOLERANCE)
}

fn line_edge<C>(front: &Vertex<Point3>, back: &Vertex<Point3>) -> Edge<Point3, C>
where Line<Point3>: ToSameGeometry<C> {
    Edge::new(
        front,
        back,
        Line(front.point(), back.point()).to_same_geometry(),
    )
}

fn axis_aligned_cuboid<C, S>(bounds: BoundingBox<Point3>) -> ShapeOpsResult<Solid<Point3, C, S>>
where
    Line<Point3>: ToSameGeometry<C>,
    Plane: ToSameGeometry<S>, {
    let p = bounds.min();
    let q = bounds.max();
    let vertices = [
        Point3::new(p.x, p.y, p.z),
        Point3::new(q.x, p.y, p.z),
        Point3::new(q.x, q.y, p.z),
        Point3::new(p.x, q.y, p.z),
        Point3::new(p.x, p.y, q.z),
        Point3::new(q.x, p.y, q.z),
        Point3::new(q.x, q.y, q.z),
        Point3::new(p.x, q.y, q.z),
    ]
    .map(Vertex::new);
    let edges = [
        line_edge(&vertices[0], &vertices[1]),
        line_edge(&vertices[1], &vertices[2]),
        line_edge(&vertices[2], &vertices[3]),
        line_edge(&vertices[3], &vertices[0]),
        line_edge(&vertices[0], &vertices[4]),
        line_edge(&vertices[1], &vertices[5]),
        line_edge(&vertices[2], &vertices[6]),
        line_edge(&vertices[3], &vertices[7]),
        line_edge(&vertices[4], &vertices[5]),
        line_edge(&vertices[5], &vertices[6]),
        line_edge(&vertices[6], &vertices[7]),
        line_edge(&vertices[7], &vertices[4]),
    ];

    let bottom = wire![
        edges[3].inverse(),
        edges[2].inverse(),
        edges[1].inverse(),
        edges[0].inverse(),
    ];
    let bottom_plane = Plane::new(
        vertices[0].point(),
        vertices[3].point(),
        vertices[1].point(),
    );
    let side_faces = (0..4).map(|index| {
        let wire = wire![
            edges[index].clone(),
            edges[(index + 1) % 4 + 4].clone(),
            edges[index + 8].inverse(),
            edges[index + 4].inverse(),
        ];
        let plane = Plane::new(
            vertices[index].point(),
            vertices[index + 1].point(),
            vertices[index + 4].point(),
        );
        Face::new_unchecked(vec![wire], plane.to_same_geometry())
    });
    let top = wire![
        edges[8].clone(),
        edges[9].clone(),
        edges[10].clone(),
        edges[11].clone(),
    ];
    let top_plane = Plane::new(
        vertices[4].point(),
        vertices[5].point(),
        vertices[7].point(),
    );
    let shell = shell![Face::new_unchecked(
        vec![bottom],
        bottom_plane.to_same_geometry()
    )];
    let shell = side_faces
        .chain([Face::new_unchecked(vec![top], top_plane.to_same_geometry())])
        .fold(shell, |mut shell, face| {
            shell.push(face);
            shell
        });
    Solid::try_new(vec![shell]).map_err(|source| ShapeOpsError::InvalidOutputShell {
        operation: HALF_SPACE_CLIP_OPERATION,
        source,
    })
}

fn finite_half_space_cuboid<C, S>(
    bounds: BoundingBox<Point3>,
    keep_positive_z: bool,
    tol: f64,
) -> ShapeOpsResult<Solid<Point3, C, S>>
where
    Line<Point3>: ToSameGeometry<C>,
    Plane: ToSameGeometry<S>,
{
    let min = bounds.min();
    let max = bounds.max();
    let margin = half_space_clip_margin(bounds, tol);
    let z_range = if keep_positive_z {
        (0.0, max.z.max(0.0) + margin)
    } else {
        (min.z.min(0.0) - margin, 0.0)
    };
    let clip_bounds = BoundingBox::from_iter([
        Point3::new(min.x - margin, min.y - margin, z_range.0),
        Point3::new(max.x + margin, max.y + margin, z_range.1),
    ]);
    axis_aligned_cuboid(clip_bounds)
}

/// Clips `solid` against a canonical clip-space half-space.
///
/// `world_to_clip` maps input coordinates into clip space, where the cutting
/// plane is `z = 0`. If `keep_positive_z` is `true`, the result keeps
/// `z >= 0`; otherwise it keeps `z <= 0`.
///
/// This is an explicit inspection operator. It is not used as a hidden
/// shape-specific rescue path by generic Boolean operations.
pub fn clip_half_space_z<C, S>(
    solid: &Solid<Point3, C, S>,
    world_to_clip: Matrix4,
    keep_positive_z: bool,
    tol: f64,
) -> ShapeOpsResult<Solid<Point3, C, S>>
where
    C: CuttableTrimAwareShapeOpsCurve<S> + Transformed<Matrix4>,
    S: ShapeOpsSurface + Transformed<Matrix4>,
    Plane: IncludeCurve<C> + ToSameGeometry<S>,
    Line<Point3>: ToSameGeometry<C>,
    <C as ExactParameterBoundary2D<S>>::BoundaryCurve: BoundedCurve
        + BoundaryCurveFromSamples<S>
        + Cut
        + Clone
        + Invertible
        + ExactTrimBoundary2D
        + Parallelizable,
{
    if tol < TOLERANCE {
        return Err(ShapeOpsError::InvalidTolerance);
    }
    let clip_to_world = world_to_clip
        .invert()
        .ok_or(ShapeOpsError::NonInvertibleClipTransform)?;
    let clip_solid = transformed_solid(solid, world_to_clip);
    let bounds = solid_bounds(&clip_solid);
    if bounds.is_empty() || half_space_discards_bounds(bounds, keep_positive_z, tol) {
        return Err(ShapeOpsError::EmptyOutputShell {
            operation: HALF_SPACE_CLIP_OPERATION,
        });
    }
    if half_space_contains_bounds(bounds, keep_positive_z, tol) {
        return Ok(solid.clone());
    }
    let clip_cuboid = finite_half_space_cuboid(bounds, keep_positive_z, tol)?;
    let clipped = and_with_orientation_hints(
        &clip_solid,
        &clip_cuboid,
        ShellOrientationHints {
            first_inverted: false,
            second_inverted: false,
        },
        tol,
    )?;
    Ok(transformed_solid(&clipped, clip_to_world))
}

/// Angular tolerance (as an in-plane component of a unit normal) for calling a
/// face's clip-space plane "parallel to the cutting plane". A genuine cap
/// inherits the axis-aligned clip cuboid's `z = 0` face and round-trips to a
/// deviation near `1e-12`; a merely tilted face is off by tenths. `1e-6`
/// (a micro-radian) separates them with an enormous margin.
const PLANE_CUT_COINCIDENCE_NORMAL_TOL: f64 = 1.0e-6;

/// The result of a planar cut through a solid: the half-space-clipped solid
/// plus the interior cross-section (cap) faces that lie on the cutting plane.
///
/// Both fields are in WORLD coordinates. `section` is the set of faces of
/// `solid` whose supporting plane is the cutting plane -- the flat caps the
/// plane sliced through the material. It is empty when the plane does not pass
/// through the solid's interior (the kept half-space wholly contains the solid,
/// with no boundary face grazing the plane).
#[derive(Clone, Debug)]
pub struct PlaneCut<C, S> {
    /// The half-space-clipped solid (world coordinates), identical to
    /// [`clip_half_space_z`]'s output.
    pub solid: Solid<Point3, C, S>,
    /// The interior cross-section: the cap faces coincident with the cutting
    /// plane, in world coordinates.
    pub section: Vec<Face<Point3, C, S>>,
}

/// True when `face`'s supporting surface is an analytic [`Plane`] coincident
/// with the cutting plane (`z = 0` in clip space): its clip-space normal is
/// parallel to `z` and its clip-space offset is within `tol` of zero.
///
/// Only planar caps can be a cross-section face; curved faces (cylinder walls,
/// sphere patches) never extract as a `Plane`, so they are rejected without a
/// coincidence test. A face merely trimmed by the cut (e.g. a cube side wall,
/// whose cut edge lies on the plane) is a vertical plane in clip space and
/// fails the parallel test, so it too is rejected.
fn face_on_clip_plane<C, S: ShapeOpsSurface>(
    face: &Face<Point3, C, S>,
    world_to_clip: Matrix4,
    tol: f64,
) -> bool {
    let Some(AnalyticSurfaceKind::Plane(plane)) = face.surface().try_into_analytic_surface_kind()
    else {
        return false;
    };
    let clip_plane = plane.transformed(world_to_clip);
    // Parallel to `z = 0`: the clip-space normal has no x/y component
    // (`|n_z| ~ 1`). `hypot` is the magnitude of the in-plane part.
    let normal = clip_plane.normal();
    let parallel_to_cut = normal.x.hypot(normal.y) <= PLANE_CUT_COINCIDENCE_NORMAL_TOL;
    // On `z = 0`: since the plane is now horizontal, every point shares one z,
    // so its origin's clip z is the whole plane's offset.
    let on_cut = clip_plane.origin().z.abs() <= tol;
    parallel_to_cut && on_cut
}

/// Cuts `solid` by a plane, returning BOTH the half-space-clipped solid AND the
/// planar cross-section (the interior cap faces the plane sliced through the
/// material).
///
/// `world_to_clip` maps input coordinates into clip space, where the cutting
/// plane is `z = 0`; `keep_positive_z` selects which half-space survives (`true`
/// keeps `z >= 0`). Build `world_to_clip` from a cutting plane's point `p` and
/// unit normal `n` as a rigid frame whose z-axis is `n` and whose origin sits on
/// the plane, so a world point's clip z equals `n . (x - p)`.
///
/// The clipped solid is produced by [`clip_half_space_z`] and therefore inherits
/// its typed-refusal guards (spec 006 Stage 3 + PR #56): a cut the Boolean
/// cannot resolve returns a typed [`ShapeOpsError`], never a silently wrong or
/// open solid.
///
/// The cross-section is extracted from the clipped solid by [`face_on_clip_plane`]:
/// the faces whose supporting surface is an analytic [`Plane`] coincident with
/// the cutting plane. They are the interior caps, returned in world coordinates
/// (the extraction only filters; the result solid is already in world space).
/// When the kept half-space wholly contains the solid the section is empty.
///
/// This is a correctness-first composition over the existing half-space
/// intersection. A bespoke single-plane intersector would avoid the cuboid
/// Boolean and be substantially faster; that is deferred as a Stage-4 perf
/// follow-up.
pub fn plane_cut<C, S>(
    solid: &Solid<Point3, C, S>,
    world_to_clip: Matrix4,
    keep_positive_z: bool,
    tol: f64,
) -> ShapeOpsResult<PlaneCut<C, S>>
where
    C: CuttableTrimAwareShapeOpsCurve<S> + Transformed<Matrix4>,
    S: ShapeOpsSurface + Transformed<Matrix4>,
    Plane: IncludeCurve<C> + ToSameGeometry<S>,
    Line<Point3>: ToSameGeometry<C>,
    <C as ExactParameterBoundary2D<S>>::BoundaryCurve: BoundedCurve
        + BoundaryCurveFromSamples<S>
        + Cut
        + Clone
        + Invertible
        + ExactTrimBoundary2D
        + Parallelizable,
{
    let cut = clip_half_space_z(solid, world_to_clip, keep_positive_z, tol)?;
    let section = cut
        .face_iter()
        .filter(|face| face_on_clip_plane(face, world_to_clip, tol))
        .cloned()
        .collect();
    Ok(PlaneCut {
        solid: cut,
        section,
    })
}

/// OR operation between two solids.
pub fn or<C: CuttableTrimAwareShapeOpsCurve<S>, S: ShapeOpsSurface>(
    solid0: &Solid<Point3, C, S>,
    solid1: &Solid<Point3, C, S>,
    tol: f64,
) -> ShapeOpsResult<Solid<Point3, C, S>>
where
    Plane: IncludeCurve<C> + ToSameGeometry<S>,
    Line<Point3>: ToSameGeometry<C>,
    <C as ExactParameterBoundary2D<S>>::BoundaryCurve: BoundedCurve
        + BoundaryCurveFromSamples<S>
        + Cut
        + Clone
        + Invertible
        + ExactTrimBoundary2D
        + Parallelizable,
{
    #[cfg(feature = "marching-ssi")]
    {
        classic::or(solid0, solid1, tol)
    }
    #[cfg(not(feature = "marching-ssi"))]
    {
        let _ = (solid0, solid1, tol);
        Err(ShapeOpsError::NoBackend)
    }
}

/// Difference: the region inside `solid0` but outside `solid1`.
pub fn difference<C: CuttableTrimAwareShapeOpsCurve<S>, S: ShapeOpsSurface>(
    solid0: &Solid<Point3, C, S>,
    solid1: &Solid<Point3, C, S>,
    tol: f64,
) -> ShapeOpsResult<Solid<Point3, C, S>>
where
    Plane: IncludeCurve<C> + ToSameGeometry<S>,
    Line<Point3>: ToSameGeometry<C>,
    <C as ExactParameterBoundary2D<S>>::BoundaryCurve: BoundedCurve
        + BoundaryCurveFromSamples<S>
        + Cut
        + Clone
        + Invertible
        + ExactTrimBoundary2D
        + Parallelizable,
{
    #[cfg(feature = "marching-ssi")]
    {
        classic::difference(solid0, solid1, tol)
    }
    #[cfg(not(feature = "marching-ssi"))]
    {
        let _ = (solid0, solid1, tol);
        Err(ShapeOpsError::NoBackend)
    }
}

const NORMALIZED_DIAMETER: f64 = 100.0;
const NORMALIZED_TOLERANCES: [f64; 5] = [2.0e-3, 4.0e-3, 1.0e-3, 1.0e-2, 4.0e-4];
const RAW_RELATIVE_TOLERANCES: [f64; 3] = [2.0e-5, 2.0e-6, 1.0e-4];
const VOLUME_SLACK: f64 = 1.0e-3;

#[derive(Clone, Copy)]
enum BooleanKind {
    And,
    Or,
    Difference,
}

fn signed_volume<C: ShapeOpsCurve<S>, S: ShapeOpsSurface>(
    solid: &Solid<Point3, C, S>,
    tol: f64,
) -> f64 {
    use monstertruck_meshing::prelude::*;
    solid
        .boundaries()
        .iter()
        .map(|shell| shell.triangulation(tol).to_polygon().volume())
        .sum()
}

fn plausible_volume(kind: BooleanKind, a: f64, b: f64, result: f64) -> bool {
    let slack = VOLUME_SLACK * a.abs().max(b.abs());
    let (low, high) = match kind {
        BooleanKind::And if b < 0.0 => (a + b, a),
        BooleanKind::And => (0.0, a.min(b)),
        BooleanKind::Or => (a.max(b), a + b),
        BooleanKind::Difference => (a - b, a),
    };
    result > low - slack && result < high + slack && result > slack
}

fn boolean_normalized<C, S>(
    kind: BooleanKind,
    solid0: &Solid<Point3, C, S>,
    solid1: &Solid<Point3, C, S>,
) -> ShapeOpsResult<Solid<Point3, C, S>>
where
    C: CuttableTrimAwareShapeOpsCurve<S> + Transformed<Matrix4>,
    S: ShapeOpsSurface + Transformed<Matrix4>,
    Plane: IncludeCurve<C> + ToSameGeometry<S>,
    Line<Point3>: ToSameGeometry<C>,
    <C as ExactParameterBoundary2D<S>>::BoundaryCurve: BoundedCurve
        + BoundaryCurveFromSamples<S>
        + Cut
        + Clone
        + Invertible
        + ExactTrimBoundary2D
        + Parallelizable,
{
    let bounds: BoundingBox<Point3> = solid0
        .vertex_iter()
        .chain(solid1.vertex_iter())
        .map(|vertex| vertex.point())
        .collect();
    let scale = NORMALIZED_DIAMETER / bounds.diameter().max(TOLERANCE);
    let to_unit = Matrix4::from_scale(scale) * Matrix4::from_translation(-bounds.center().to_vec());
    let from_unit =
        Matrix4::from_translation(bounds.center().to_vec()) * Matrix4::from_scale(1.0 / scale);
    let unit0 = transformed_solid(solid0, to_unit);
    let unit1 = transformed_solid(solid1, to_unit);
    let measure = NORMALIZED_TOLERANCES[0] * 10.0;
    let unit_volumes = (
        signed_volume(&unit0, measure),
        signed_volume(&unit1, measure),
    );
    let diameter = bounds.diameter().max(TOLERANCE);
    let raw_measure = measure * diameter / NORMALIZED_DIAMETER;
    let cube = scale.powi(3);
    let raw_volumes = (unit_volumes.0 / cube, unit_volumes.1 / cube);
    let attempts: Vec<(bool, f64)> = NORMALIZED_TOLERANCES
        .iter()
        .map(|&tol| (true, tol))
        .zip(
            RAW_RELATIVE_TOLERANCES
                .iter()
                .map(|&k| Some((false, k * diameter)))
                .chain(repeat(None)),
        )
        .flat_map(|(normalized, raw)| once(normalized).chain(raw))
        .collect();
    let attempt =
        |&(normalized, tol): &(bool, f64)| -> ShapeOpsResult<Option<Solid<Point3, C, S>>> {
            let (a, b, volumes, measure) = match normalized {
                true => (&unit0, &unit1, unit_volumes, measure),
                false => (solid0, solid1, raw_volumes, raw_measure),
            };
            let solid = match kind {
                BooleanKind::And => and(a, b, tol),
                BooleanKind::Or => or(a, b, tol),
                BooleanKind::Difference => difference(a, b, tol),
            }?;
            let plausible =
                plausible_volume(kind, volumes.0, volumes.1, signed_volume(&solid, measure));
            Ok(plausible.then(|| match normalized {
                true => transformed_solid(&solid, from_unit),
                false => solid,
            }))
        };
    let mut last_error = ShapeOpsError::EmptyOutputShell {
        operation: "normalized boolean",
    };
    for outcome in attempts.iter().map(attempt) {
        match outcome {
            Ok(Some(solid)) => return Ok(solid),
            Ok(None) => {}
            Err(error) => last_error = error,
        }
    }
    Err(last_error)
}

macro_rules! normalized_boolean {
    ($(#[$doc:meta])* $name:ident, $kind:expr) => {
        $(#[$doc])*
        pub fn $name<C, S>(
            solid0: &Solid<Point3, C, S>,
            solid1: &Solid<Point3, C, S>,
        ) -> ShapeOpsResult<Solid<Point3, C, S>>
        where
            C: CuttableTrimAwareShapeOpsCurve<S> + Transformed<Matrix4>,
            S: ShapeOpsSurface + Transformed<Matrix4>,
            Plane: IncludeCurve<C> + ToSameGeometry<S>,
            Line<Point3>: ToSameGeometry<C>,
            <C as ExactParameterBoundary2D<S>>::BoundaryCurve: BoundedCurve
                + BoundaryCurveFromSamples<S>
                + Cut
                + Clone
                + Invertible
                + ExactTrimBoundary2D
                + Parallelizable,
        {
            boolean_normalized($kind, solid0, solid1)
        }
    };
}

normalized_boolean!(
    /// [`and`] on copies scaled to a fixed size, retried across tolerances until the result volume is consistent with the inputs.
    and_normalized,
    BooleanKind::And
);
normalized_boolean!(
    /// [`or`] on copies scaled to a fixed size, retried across tolerances until the result volume is consistent with the inputs.
    ///
    /// # Examples
    ///
    /// ```
    /// use monstertruck_modeling::*;
    /// # fn main() -> std::result::Result<(), Box<dyn std::error::Error>> {
    /// let cube = |min: Point3, max: Point3| -> Solid { primitive::cuboid(BoundingBox::from_iter([min, max])) };
    /// let a = cube(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 2.0));
    /// let b = cube(Point3::new(1.0, 1.0, 1.0), Point3::new(3.0, 3.0, 3.0));
    /// let solid = monstertruck_solid::or_normalized(&a, &b)?;
    /// assert_eq!(solid.boundaries().len(), 1);
    /// # Ok(())
    /// # }
    /// ```
    or_normalized,
    BooleanKind::Or
);
normalized_boolean!(
    /// [`difference`] on copies scaled to a fixed size, retried across tolerances until the result volume is consistent with the inputs.
    difference_normalized,
    BooleanKind::Difference
);

/// Symmetric difference (XOR): the region inside exactly one of the solids.
pub fn symmetric_difference<C: CuttableTrimAwareShapeOpsCurve<S>, S: ShapeOpsSurface>(
    solid0: &Solid<Point3, C, S>,
    solid1: &Solid<Point3, C, S>,
    tol: f64,
) -> ShapeOpsResult<Solid<Point3, C, S>>
where
    Plane: IncludeCurve<C> + ToSameGeometry<S>,
    Line<Point3>: ToSameGeometry<C>,
    <C as ExactParameterBoundary2D<S>>::BoundaryCurve: BoundedCurve
        + BoundaryCurveFromSamples<S>
        + Cut
        + Clone
        + Invertible
        + ExactTrimBoundary2D
        + Parallelizable,
{
    #[cfg(feature = "marching-ssi")]
    {
        classic::symmetric_difference(solid0, solid1, tol)
    }
    #[cfg(not(feature = "marching-ssi"))]
    {
        let _ = (solid0, solid1, tol);
        Err(ShapeOpsError::NoBackend)
    }
}

#[cfg(test)]
mod tests;
