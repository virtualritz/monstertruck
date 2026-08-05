//! Round-trip guard for planar-cap parameter curves through STEP.
//!
//! A cylinder solid's two planar caps carry `NurbsCurve` trim arcs. Once
//! `compress_with_parameter_curves` attaches a face-local parameter curve to
//! each cap arc, saving to STEP must emit a `PCURVE` for those arcs and
//! reloading must preserve the two planar cap faces with their trim data. A
//! regression that dropped the cap pcurves made the caps disappear from a
//! downstream NURBS export whose parameter-curve fallback could not recover
//! them.

use monstertruck_io::step::load::{Table, step_geometry};
use monstertruck_io::step::save::*;
use monstertruck_modeling::*;

/// Era-standard modeling-layer right circular cylinder.
fn cylinder(height: f64, radius: f64) -> Solid {
    let vertex = builder::vertex(Point3::new(0.0, -height / 2.0, radius));
    let circle = builder::revolve(
        &vertex,
        Point3::origin(),
        Vector3::unit_y(),
        builder::SweepAngle::Closed,
        2,
    );
    let disk = builder::try_attach_plane(&[circle]).unwrap();
    builder::extrude(&disk, Vector3::new(0.0, height, 0.0))
}

fn is_planar(surface: &step_geometry::Surface) -> bool {
    matches!(
        surface,
        step_geometry::Surface::ElementarySurface(step_geometry::ElementarySurface::Plane(_))
    )
}

#[test]
fn cylinder_planar_caps_survive_step_roundtrip() {
    let solid = cylinder(1.0, 0.5);
    let tolerance = 0.01;
    let compressed = solid.compress_with_parameter_curves(tolerance);

    // Save the trimmed solid (with parameter curves) to a STEP file.
    let step_text =
        CompleteStepDisplay::new(StepModel::from(&compressed), Default::default()).to_string();

    // Sanity: the trim machinery serialized parameter curves at all. This is a
    // weak global count (lateral faces contribute PCURVEs too, so it does not by
    // itself prove the caps are covered); the discriminating guard is the reload
    // check below, which reconstructs the cap faces and asserts their trims. For
    // this cylinder the fix raises the total from 8 to 12 (four extra cap-arc
    // PCURVEs: two caps x two arcs), but the exact total is left unasserted to
    // avoid brittleness against unrelated serializer changes.
    let pcurve_count = step_text.matches("PCURVE(").count();
    assert!(
        pcurve_count > 0,
        "STEP output should emit PCURVE entities for the trimmed solid, got {pcurve_count}",
    );

    // Reload and confirm the two planar cap faces survive with trim curves.
    let table = Table::from_step(&step_text).expect("emitted STEP must parse and index");
    let step_shell = table
        .shell
        .values()
        .next()
        .expect("reloaded cylinder must contain a shell");
    let reloaded = table
        .to_compressed_trimmed_shell(step_shell)
        .expect("reloaded shell must build a trimmed shell");

    let planar_faces: Vec<_> = reloaded
        .faces
        .iter()
        .filter(|face| is_planar(&face.surface))
        .collect();
    assert_eq!(
        planar_faces.len(),
        2,
        "both planar cap faces must survive the STEP round-trip",
    );

    for (cap_index, cap) in planar_faces.iter().enumerate() {
        let edge_uses: Vec<_> = cap.boundaries.iter().flat_map(|wire| wire.iter()).collect();
        let total = edge_uses.len();
        let trims_present = edge_uses
            .iter()
            .filter(|edge_use| edge_use.trim_curve.is_some())
            .count();
        // Every boundary arc of a reloaded cap must carry its parameter curve.
        // Before the fix the caps came back with zero trims (0/total); the fix
        // restores a trim on every cap edge-use (total/total).
        assert!(
            total > 0 && trims_present == total,
            "planar cap face {cap_index} must keep a trim curve on every boundary arc \
             across the STEP round-trip, got {trims_present}/{total}",
        );
    }
}
