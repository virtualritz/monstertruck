//! Spheres on cube corners -- boolean operations.
//!
//! Places 1/3-unit spheres on all 8 corners of a unit cube. Four spheres on
//! one tetrahedral diagonal are subtracted, the other four are unioned.
//! Outputs both JSON and STEP files.

use anyhow::Result;
use monstertruck::example_output;
use monstertruck_geometry::prelude::{
    BoundedCurve, ExactParameterBoundary2D, ParameterCurve, Point2, SearchNearestParameter,
    SearchParameter,
};
use monstertruck_io::step::save::{CompleteStepDisplay, StepModel};
use monstertruck_mesh::PolylineCurve as PolylineCurve2;
use monstertruck_modeling::*;
use monstertruck_solid::{difference, or};
use monstertruck_topology::compress::{CompressedSolid, CompressedTrimmedSolid};
use std::f64::consts::PI;
use std::path::PathBuf;

const CHECKPOINT_NAME: &str = "filleted-spheres-cube";

type ExportTrimCurve = ParameterCurve<Curve2D, Box<Surface>>;

#[derive(Clone, Copy)]
enum Operation {
    Difference(Point3),
    Union(Point3),
}

/// STEP export modes for example artifacts.
///
/// Set via `MT_STEP_EXPORT_FLAVOR`.
///
/// The default is [`StepExportFlavor::Curve3DOnly`], which preserves the older
/// viewer-friendly shared-edge path.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum StepExportFlavor {
    /// Writes shared edge geometry without face-local trims.
    ///
    /// `ParameterCurve` values are lowered to 3-dimensional carriers.
    /// `IntersectionCurve` values stay shared as simplified
    /// `INTERSECTION_CURVE` entities, but any face-local trim baggage is
    /// stripped.
    ///
    /// This best matches the older watertight viewer path.
    Curve3DOnly,
    /// Writes only shared 3D edge curves and face surfaces.
    ///
    /// This keeps shared topology edges authoritative but still preserves any
    /// associated curve semantics already present on the exported edge curves.
    Shared,
    /// Writes only exact face-local trims that are already available on the
    /// edge curve itself.
    ///
    /// This preserves exact trim data that already exists and leaves all other
    /// coedges on the shared-edge path.
    ExactTrims,
    /// Writes face-local trims for every coedge.
    ///
    /// When an exact trim is unavailable, this mode samples the 3D edge and
    /// projects the samples into the face parameter domain.
    ///
    /// This mode is opt-in because independently sampled coedge trims can
    /// diverge across adjacent faces until the kernel preserves exact shared
    /// trims end-to-end.
    FaceTrims,
}

/// Resolves the STEP export flavor from `MT_STEP_EXPORT_FLAVOR`.
///
/// Accepted values are `"curve-3d-only"`, `"shared"`, `"exact-trims"`,
/// and `"face-trims"`.
///
/// Any other value falls back to the conservative shared-edge export.
fn step_export_flavor() -> StepExportFlavor {
    match std::env::var("MT_STEP_EXPORT_FLAVOR").ok().as_deref() {
        Some("curve-3d-only") => StepExportFlavor::Curve3DOnly,
        Some("shared") => StepExportFlavor::Shared,
        Some("exact-trims") => StepExportFlavor::ExactTrims,
        Some("face-trims") => StepExportFlavor::FaceTrims,
        _ => StepExportFlavor::Curve3DOnly,
    }
}

fn curve3d_only_curve(curve: &Curve) -> Curve {
    match curve {
        Curve::Line(_) | Curve::BsplineCurve(_) | Curve::NurbsCurve(_) => curve.clone(),
        Curve::ParameterCurve(parameter_curve) => Curve::NurbsCurve(NurbsCurve::new(
            Curve::ParameterCurve(parameter_curve.clone()).lift_up(),
        )),
        Curve::IntersectionCurve(surface_curve) => {
            Curve::IntersectionCurve(SurfaceCurve::with_boundaries(
                surface_curve.surface0().clone(),
                surface_curve.surface1().clone(),
                Box::new(curve3d_only_curve(surface_curve.leader())),
                None,
                None,
            ))
        }
    }
}

fn write_final_artifacts(result: &Solid, trim_tolerance: f64) -> Result<()> {
    let json = serde_json::to_vec_pretty(result)?;
    std::fs::write(
        example_output::artifact_path("filleted-spheres-cube.json")?,
        &json,
    )?;
    let shared: CompressedSolid<_, _, _> = result.compress();
    let curve3d_only = shared.map_curves(curve3d_only_curve);
    let curve3d_only_step = CompleteStepDisplay::new(
        StepModel::from_curve3d_only_solid(&curve3d_only),
        Default::default(),
    )
    .to_string();
    let shared_step =
        CompleteStepDisplay::new(StepModel::from(&shared), Default::default()).to_string();
    let exact: CompressedTrimmedSolid<_, _, _, _> =
        result.compress_with_face_trims(exact_face_trim);
    let exact_step =
        CompleteStepDisplay::new(StepModel::from(&exact), Default::default()).to_string();
    let face_trims: CompressedTrimmedSolid<_, _, _, _> = result
        .compress_with_face_trims(|edge, surface| sampled_face_trim(edge, surface, trim_tolerance));
    let face_trims_step =
        CompleteStepDisplay::new(StepModel::from(&face_trims), Default::default()).to_string();
    std::fs::write(
        example_output::artifact_path("filleted-spheres-cube-curve3d-only.step")?,
        &curve3d_only_step,
    )?;
    std::fs::write(
        example_output::artifact_path("filleted-spheres-cube-shared.step")?,
        &shared_step,
    )?;
    std::fs::write(
        example_output::artifact_path("filleted-spheres-cube-exact-trims.step")?,
        &exact_step,
    )?;
    std::fs::write(
        example_output::artifact_path("filleted-spheres-cube-face-trims.step")?,
        &face_trims_step,
    )?;
    let canonical = match step_export_flavor() {
        StepExportFlavor::Curve3DOnly => &curve3d_only_step,
        StepExportFlavor::Shared => &shared_step,
        StepExportFlavor::ExactTrims => &exact_step,
        StepExportFlavor::FaceTrims => &face_trims_step,
    };
    std::fs::write(
        example_output::artifact_path("filleted-spheres-cube.step")?,
        canonical,
    )?;
    Ok(())
}

fn exact_face_trim(edge: &Edge, surface: &Surface) -> Option<ExportTrimCurve> {
    edge.curve().exact_parameter_boundary_2d(surface)
}

fn sampled_face_trim(edge: &Edge, surface: &Surface, tolerance: f64) -> Option<ExportTrimCurve> {
    exact_face_trim(edge, surface).or_else(|| {
        let curve = edge.curve();
        let (_, mut points) = curve.parameter_division(curve.range_tuple(), tolerance);
        if points.is_empty() {
            points = vec![curve.front(), curve.back()];
        }
        let mut hint = None;
        let mut boundary = points
            .into_iter()
            .map(|point| {
                let uv = surface
                    .search_parameter(point, hint.map(Into::into), 50)
                    .or_else(|| surface.search_nearest_parameter(point, hint.map(Into::into), 50))
                    .or_else(|| surface.search_parameter(point, None, 50))
                    .or_else(|| surface.search_nearest_parameter(point, None, 50))
                    .map(Point2::from)?;
                hint = Some(uv);
                Some(uv)
            })
            .collect::<Option<Vec<_>>>()?;
        if boundary.len() < 2 {
            None
        } else {
            let first = boundary[0];
            let last = *boundary.last()?;
            if !first.near(&last) {
                boundary.push(first);
            }
            Some(ParameterCurve::new(
                Curve2D::Polyline(PolylineCurve2(boundary)),
                Box::new(surface.clone()),
            ))
        }
    })
}

fn checkpoint_dir() -> Result<PathBuf> {
    let dir = example_output::artifact_dir()?
        .join("checkpoints")
        .join(CHECKPOINT_NAME);
    std::fs::create_dir_all(&dir)?;
    Ok(dir)
}

fn checkpoint_path(step: usize) -> Result<PathBuf> {
    checkpoint_dir().map(|dir| dir.join(format!("step-{step:02}.json")))
}

fn write_checkpoint(step: usize, result: &Solid) -> Result<()> {
    let json = serde_json::to_vec(result)?;
    std::fs::write(checkpoint_path(step)?, json)?;
    Ok(())
}

fn load_latest_checkpoint() -> Result<Option<(usize, Solid)>> {
    (0..=8)
        .rev()
        .find_map(|step| {
            checkpoint_path(step)
                .ok()
                .filter(|path| path.exists())
                .map(|path| (step, path))
        })
        .map(|(step, path)| {
            std::fs::read(path)
                .map_err(Into::into)
                .and_then(|bytes| serde_json::from_slice(&bytes).map_err(Into::into))
                .map(|solid| (step, solid))
        })
        .transpose()
}

fn cleanup_checkpoints() -> Result<()> {
    let dir = checkpoint_dir()?;
    if dir.exists() {
        std::fs::remove_dir_all(dir)?;
    }
    Ok(())
}

fn max_step_override() -> Option<usize> {
    std::env::var("MT_FILLETED_MAX_STEP")
        .ok()
        .and_then(|value| value.parse::<usize>().ok())
}

/// Create a sphere solid centered at `center` with the given `radius`.
fn sphere(center: Point3, radius: f64) -> Solid {
    let top = builder::vertex(Point3::new(0.0, radius, 0.0));
    let wire: Wire = builder::revolve(
        &top,
        Point3::origin(),
        Vector3::unit_x(),
        builder::SweepAngle::Partial(Rad(PI)),
        3,
    );
    let shell = builder::revolve_wire(
        &wire,
        Point3::origin(),
        Vector3::unit_y(),
        builder::SweepAngle::Closed,
        4,
    );
    let s = Solid::new(vec![shell]);
    builder::translated(&s, center.to_vec())
}

fn main() -> Result<()> {
    let tol = 0.01;
    let r = 1.0 / 3.0;

    // Unit cube at origin.
    let v = builder::vertex(Point3::origin());
    let e = builder::extrude(&v, Vector3::unit_x());
    let f = builder::extrude(&e, Vector3::unit_y());
    let cube = builder::extrude(&f, Vector3::unit_z());

    // Tetrahedral group A -- subtract these four corners.
    let subtract = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(1.0, 0.0, 1.0),
        Point3::new(0.0, 1.0, 1.0),
    ];

    // Tetrahedral group B -- union these four corners.
    let unite = [
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(0.0, 0.0, 1.0),
        Point3::new(1.0, 1.0, 1.0),
    ];

    let operations = subtract
        .into_iter()
        .map(Operation::Difference)
        .chain(unite.into_iter().map(Operation::Union))
        .collect::<Vec<_>>();

    let (start_step, mut body) = load_latest_checkpoint()?.unwrap_or((0, cube));
    if start_step == 0 {
        write_checkpoint(0, &body)?;
    }

    let max_step = max_step_override();

    operations
        .iter()
        .enumerate()
        .skip(start_step)
        .try_for_each(|(index, operation)| {
            if max_step.is_some_and(|step| index >= step) {
                return Ok(());
            }
            let (op_name, center, next) = match *operation {
                Operation::Difference(center) => {
                    let sphere = sphere(center, r);
                    ("difference", center, difference(&body, &sphere, tol))
                }
                Operation::Union(center) => {
                    let sphere = sphere(center, r);
                    ("union", center, or(&body, &sphere, tol))
                }
            };
            if let Ok(next) = next {
                body = next;
                write_checkpoint(index + 1, &body)?;
                Ok(())
            } else if let Err(error) = next {
                Err(anyhow::anyhow!(
                    "{op_name} failed at step {} center {center:?}: {error}",
                    index + 1
                ))
            } else {
                unreachable!()
            }
        })?;

    write_final_artifacts(&body, tol)?;
    cleanup_checkpoints()?;

    println!(
        "Wrote {} and {}",
        example_output::artifact_path("filleted-spheres-cube.json")?.display(),
        example_output::artifact_path("filleted-spheres-cube.step")?.display(),
    );
    Ok(())
}
