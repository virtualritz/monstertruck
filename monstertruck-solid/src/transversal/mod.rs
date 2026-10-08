// The classic (0.3.2) boolean pipeline: the self-contained default backend.
// Compiled only when the marching SSI backend is enabled (the published
// default). Under `--no-default-features` it is absent and the boolean entry
// points in `integrate` return `ShapeOpsError::NoBackend`.
#[cfg(feature = "marching-ssi")]
mod classic;
mod integrate;
// Marching-SSI polyline chaining support consumed by the classic backend.
#[cfg(feature = "marching-ssi")]
mod polyline_construction;
pub use integrate::{
    PlaneCut, ShapeOpsCurve, ShapeOpsError, ShapeOpsSurface, ShellOrientationHints, and,
    and_normalized, and_with_orientation_hints, clip_half_space_z, difference,
    difference_normalized, or, or_normalized, plane_cut, symmetric_difference,
};
pub use monstertruck_traits::SnapCurveEndpoints;
