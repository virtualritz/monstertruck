//! Edge associations reference the face's surface entity instead of
//! re-emitting a full copy per edge.
//!
//! The default shell writer associates every edge with the surfaces of its
//! adjacent faces via `SURFACE_CURVE`. Historically each association re-emitted
//! the whole surface entity, so a surface appeared once as the face geometry
//! plus twice per bounding edge (~3x bloat on surface-heavy shells). Each
//! surface must now be emitted exactly once -- as the face geometry -- and the
//! `SURFACE_CURVE` associations must reference that entity.

use monstertruck_io::step::load::Table;
use monstertruck_io::step::save::*;
use monstertruck_modeling::*;

/// Builds a unit cube solid via the extrude chain. Every face is planar, so
/// the emitted surface entity is `PLANE`.
fn unit_cube() -> Solid {
    let v = builder::vertex(Point3::origin());
    let e = builder::extrude(&v, Vector3::unit_x());
    let f = builder::extrude(&e, Vector3::unit_y());
    builder::extrude(&f, Vector3::unit_z())
}

/// Counts non-overlapping occurrences of `needle` in `haystack`.
fn count(haystack: &str, needle: &str) -> usize { haystack.matches(needle).count() }

#[test]
fn surface_is_emitted_once_per_face_not_per_edge() {
    let compressed = unit_cube().compress();
    let face_count = compressed.boundaries[0].faces.len();
    assert_eq!(face_count, 6, "a cube has six faces");

    let step =
        CompleteStepDisplay::new(StepModel::from(&compressed), Default::default()).to_string();

    // The associations still exist (each shared edge lies on two faces).
    assert!(
        step.contains("SURFACE_CURVE"),
        "edges must still associate their surfaces via SURFACE_CURVE:\n{step}"
    );

    // One `PLANE` per face -- no per-edge re-emission. Before the dedup a cube
    // emitted 6 face planes plus two per edge (12 edges) = 30 planes.
    let plane_count = count(&step, "= PLANE(");
    assert_eq!(
        plane_count, face_count,
        "each face surface must be emitted exactly once (got {plane_count} PLANE \
         entities for {face_count} faces):\n{step}"
    );
}

#[test]
fn deduped_surface_curve_still_reloads() {
    let compressed = unit_cube().compress();
    let step =
        CompleteStepDisplay::new(StepModel::from(&compressed), Default::default()).to_string();

    // Structurally valid and reloads to the same face set.
    let table = Table::from_step(&step).unwrap();
    let step_shell = table
        .shell
        .values()
        .next()
        .expect("reloaded STEP has a shell");
    let reloaded = table.to_compressed_shell(step_shell).unwrap();
    assert_eq!(reloaded.faces.len(), 6, "roundtrip preserves the six faces");
}
