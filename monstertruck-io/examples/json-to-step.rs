//! Converts a serialized `monstertruck_modeling::Solid` JSON file to STEP.

use anyhow::{Context, Result};
use clap::Parser;
use monstertruck_geometry::prelude::{
    BoundedCurve, ExactParameterBoundary2D, MetricSpace, ParameterCurve, Point2,
    SearchNearestParameter, SearchParameter,
};
use monstertruck_io::step::save::{CompleteStepDisplay, StepModel};
use monstertruck_mesh::PolylineCurve as PolylineCurve2;
use monstertruck_modeling::{Curve2D, Edge, Solid, Surface};
use monstertruck_topology::compress::{CompressedSolid, CompressedTrimmedSolid};
use monstertruck_traits::ParameterDivision1D;
use std::path::PathBuf;

type ExportTrimCurve = ParameterCurve<Curve2D, Box<Surface>>;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum StepExportFlavor {
    /// Writes shared edge geometry without face-local trims.
    ///
    /// `ParameterCurve` values are lowered to 3-dimensional carriers.
    /// `IntersectionCurve` values stay shared as simplified
    /// `INTERSECTION_CURVE` entities, but any face-local trim baggage is
    /// stripped.
    Curve3DOnly,
    Shared,
    ExactTrims,
    FaceTrims,
}

fn step_export_flavor() -> StepExportFlavor {
    match std::env::var("MT_STEP_EXPORT_FLAVOR").ok().as_deref() {
        Some("curve-3d-only") => StepExportFlavor::Curve3DOnly,
        Some("shared") => StepExportFlavor::Shared,
        Some("exact-trims") => StepExportFlavor::ExactTrims,
        Some("face-trims") => StepExportFlavor::FaceTrims,
        _ => StepExportFlavor::Curve3DOnly,
    }
}

fn curve3d_only_curve(curve: &monstertruck_modeling::Curve) -> monstertruck_modeling::Curve {
    match curve {
        monstertruck_modeling::Curve::Line(_)
        | monstertruck_modeling::Curve::BsplineCurve(_)
        | monstertruck_modeling::Curve::NurbsCurve(_) => curve.clone(),
        monstertruck_modeling::Curve::ParameterCurve(parameter_curve) => {
            monstertruck_modeling::Curve::NurbsCurve(monstertruck_modeling::NurbsCurve::new(
                monstertruck_modeling::Curve::ParameterCurve(parameter_curve.clone()).lift_up(),
            ))
        }
        monstertruck_modeling::Curve::IntersectionCurve(surface_curve) => {
            monstertruck_modeling::Curve::IntersectionCurve(
                monstertruck_modeling::SurfaceCurve::with_boundaries(
                    surface_curve.surface0().clone(),
                    surface_curve.surface1().clone(),
                    Box::new(curve3d_only_curve(surface_curve.leader())),
                    None,
                    None,
                ),
            )
        }
    }
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
                    .search_parameter(point, hint, 50)
                    .or_else(|| surface.search_nearest_parameter(point, hint, 50))
                    .or_else(|| surface.search_parameter(point, None, 50))
                    .or_else(|| surface.search_nearest_parameter(point, None, 50))
                    .map(|uv| Point2::new(uv.0, uv.1))?;
                hint = Some((uv.x, uv.y));
                Some(uv)
            })
            .collect::<Option<Vec<_>>>()?;
        boundary.dedup_by(|a, b| a.distance2(*b) <= 1.0e-12);
        (boundary.len() >= 2).then(|| {
            ParameterCurve::new(
                Curve2D::Polyline(PolylineCurve2(boundary)),
                Box::new(surface.clone()),
            )
        })
    })
}

#[derive(Debug, Parser)]
struct Args {
    /// Input JSON file containing a serialized `Solid`.
    input_json_file: PathBuf,
    /// Optional output STEP file. Defaults to the input path with `.step`.
    output_step_file: Option<PathBuf>,
}

fn main() -> Result<()> {
    let Args {
        input_json_file,
        output_step_file,
    } = Args::parse();
    let json = std::fs::read(&input_json_file)
        .with_context(|| format!("failed to read {}", input_json_file.display()))?;
    let solid: Solid = serde_json::from_slice(&json)
        .with_context(|| format!("failed to parse {}", input_json_file.display()))?;
    let output = output_step_file.unwrap_or_else(|| input_json_file.with_extension("step"));
    let step = match step_export_flavor() {
        StepExportFlavor::Curve3DOnly => {
            let compressed: CompressedSolid<_, _, _> =
                solid.compress().map_curves(curve3d_only_curve);
            CompleteStepDisplay::new(
                StepModel::from_curve3d_only_solid(&compressed),
                Default::default(),
            )
            .to_string()
        }
        StepExportFlavor::Shared => {
            let compressed: CompressedSolid<_, _, _> = solid.compress();
            CompleteStepDisplay::new(StepModel::from(&compressed), Default::default()).to_string()
        }
        StepExportFlavor::ExactTrims => {
            let compressed: CompressedTrimmedSolid<_, _, _, _> =
                solid.compress_with_face_trims(exact_face_trim);
            CompleteStepDisplay::new(StepModel::from(&compressed), Default::default()).to_string()
        }
        StepExportFlavor::FaceTrims => {
            let compressed: CompressedTrimmedSolid<_, _, _, _> = solid
                .compress_with_face_trims(|edge, surface| sampled_face_trim(edge, surface, 0.01));
            CompleteStepDisplay::new(StepModel::from(&compressed), Default::default()).to_string()
        }
    };
    std::fs::write(&output, step)
        .with_context(|| format!("failed to write {}", output.display()))?;
    println!("{}", output.display());
    Ok(())
}
