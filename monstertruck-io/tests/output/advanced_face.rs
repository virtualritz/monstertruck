//! AP203/AP214/AP242 conformance: shell faces serialize as `ADVANCED_FACE`.
//!
//! An `advanced_brep`-style representation places each face directly into the
//! shell as an `advanced_face` (a `face_surface` subtype), not as an
//! `oriented_face` wrapping a bare `face_surface`. This pins that the writer
//! emits `ADVANCED_FACE` and that the collapsed entity still reloads to the
//! same face set (the face orientation is folded into `same_sense`).

use monstertruck_io::step::load::Table;
use monstertruck_io::step::save::*;
use monstertruck_modeling::*;

/// Builds a unit cube solid via the extrude chain.
fn unit_cube() -> Solid {
    let v = builder::vertex(Point3::origin());
    let e = builder::extrude(&v, Vector3::unit_x());
    let f = builder::extrude(&e, Vector3::unit_y());
    builder::extrude(&f, Vector3::unit_z())
}

#[test]
fn shell_faces_serialize_as_advanced_face() {
    let compressed = unit_cube().compress();
    let step =
        CompleteStepDisplay::new(StepModel::from(&compressed), Default::default()).to_string();

    assert!(
        step.contains("ADVANCED_FACE"),
        "expected ADVANCED_FACE in emitted STEP, got:\n{step}"
    );
    assert!(
        !step.contains("FACE_SURFACE"),
        "faces must collapse to ADVANCED_FACE, not FACE_SURFACE:\n{step}"
    );
    assert!(
        !step.contains("ORIENTED_FACE"),
        "faces must be ADVANCED_FACE placed directly in the shell, \
         not wrapped in ORIENTED_FACE:\n{step}"
    );
}

#[test]
fn advanced_face_reloads_to_the_same_face_set() {
    let compressed = unit_cube().compress();
    let face_count = compressed.boundaries[0].faces.len();
    assert_eq!(face_count, 6, "a cube has six faces");

    let step =
        CompleteStepDisplay::new(StepModel::from(&compressed), Default::default()).to_string();

    let table = Table::from_step(&step).unwrap();
    let step_shell = table
        .shell
        .values()
        .next()
        .expect("reloaded STEP has a shell");
    let reloaded = table.to_compressed_shell(step_shell).unwrap();

    assert_eq!(
        reloaded.faces.len(),
        face_count,
        "ADVANCED_FACE roundtrip must preserve the face count"
    );
}
