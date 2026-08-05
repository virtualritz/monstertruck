//! Ledger class C15 -- a divergence-theorem volume read off a shell whose faces
//! do not share an orientation.
//!
//! # The instrument this file exists to certify
//!
//! `Solid::triangulation(tol).to_polygon().volume()` is the number the kernel
//! judges its own booleans with: `OperandVolume::trusted` bounds every operand
//! with it and `verify_volume_conservation` bounds every boolean RESULT with
//! it. `OperandVolume::trusted` checks only `> 0` and `<=` the operand's own
//! bounding box -- since spec 014 W2 a CERTIFIED box
//! (`monstertruck_modeling::bounding`) rather than the vertex hull that
//! preceded it, but still a magnitude test, so it cannot see a sign defect that
//! leaves the magnitude plausible -- which means the rows below are the only
//! thing standing between a wrongly-oriented load and a boolean verdict that
//! inherits it.
//!
//! # What it asserts
//!
//! The SIGNED volume of every in-repo analytic STEP fixture against its own
//! CLOSED FORM, read off the STEP file rather than pinned:
//!
//! | fixture | entity | closed form | value |
//! |---|---|---|---|
//! | `occt-cube` | 6 `PLANE`s, `[0,10]^3` | `10^3` | 1000 |
//! | `occt-sphere` | `SPHERICAL_SURFACE('',#23,5.)` | `4/3 pi r^3` | 523.5988 |
//! | `occt-cylinder` | `CYLINDRICAL_SURFACE('',#32,2.)`, `z` in `[0,10]` | `pi r^2 h` | 125.6637 |
//! | `occt-cone` | `CONICAL_SURFACE('',#32,2.,0.19739556)`, `z` in `[0,10]` | `pi h (R^2+Rr+r^2)/3` | 293.2153 |
//! | `occt-torus` | `TOROIDAL_SURFACE('',#23,10.,2.)` | `2 pi^2 R r^2` | 789.5684 |
//!
//! plus `primitive::cuboid` as the control arm: the kernel's own constructor,
//! which never went through the STEP load path.
//!
//! The band is 0.5% and the mesh is INSCRIBED, so every row must land just
//! BELOW its closed form. Nothing here can be widened into passing: the defects
//! it was written against are SIGN flips, i.e. 200% away from the band.
//!
//! # Measured before the fixes (2026-08-01), which is why this file exists
//!
//! * `occt-cube` **-1000.000000** against +1000, and
//! * `occt-sphere` **-523.253857** against +523.5988,
//!
//! while `occt-cylinder` (+125.634), `occt-cone` (+293.167), `occt-torus`
//! (+788.962) and `primitive::cuboid` (+1000 exact) were already right. Two
//! independent mechanisms produced those two numbers -- see
//! `occt_cube_shell_is_consistently_oriented_as_loaded` below for the first,
//! and `ensure_winding_matches_normals` in `monstertruck-meshing` for the
//! second.

use monstertruck_io::step::load::{
    Table,
    step_geometry::{Curve3D, StepParameterCurve, Surface as StepSurface},
};
use monstertruck_meshing::prelude::*;
use monstertruck_modeling::*;
use monstertruck_topology::compress::CompressedTrimmedSolid;

const TOL: f64 = 1.0e-3;

/// Relative band on every closed form. The mesh is inscribed, so the measured
/// value sits just under the exact one; the coarsest row (the torus) is 0.077%
/// low at this chord.
const BAND: f64 = 5.0e-3;

type StepTrimmedSolid = CompressedTrimmedSolid<Point3, Curve3D, StepSurface, StepParameterCurve>;

fn table(name: &str) -> Table {
    let path = format!("{}/../resources/step/{name}", env!("CARGO_MANIFEST_DIR"));
    let bytes = std::fs::read(&path).unwrap_or_else(|err| panic!("{path}: {err}"));
    Table::from_step_bytes(&bytes).unwrap_or_else(|err| panic!("{path}: {err:?}"))
}

fn compressed(table: &Table) -> StepTrimmedSolid {
    let (_, holder) = table
        .manifold_solid_brep
        .iter()
        .next()
        .expect("every fixture here has exactly one MANIFOLD_SOLID_BREP");
    table
        .to_compressed_trimmed_solid(holder)
        .expect("the fixture must compress")
}

/// The production extraction path, exactly as `user_fixture_boolean_tests`'s
/// `try_extract` runs it: compress -> heal -> erase trims -> map.
fn extract(name: &str) -> Solid {
    let csolid = compressed(&table(name));
    monstertruck_healing::extract_healed_trimmed_solid(csolid, TOL)
        .unwrap_or_else(|err| panic!("{name}: must heal: {err:?}"))
        .erase_trims()
        .try_mapped(
            |point| Some(*point),
            |curve: &Curve3D| Curve::try_from(curve).ok(),
            |surface: &StepSurface| Surface::try_from(surface).ok(),
        )
        .unwrap_or_else(|| panic!("{name}: must map to modeling geometry"))
}

/// Measures one row and REPORTS it; returns the complaint if it is out of band.
/// Every row is measured before any of them fails, so one run says which rows
/// moved rather than only the first.
fn signed_volume_row(label: &str, solid: &Solid, closed_form: f64) -> Option<String> {
    let measured = solid.triangulation(TOL).to_polygon().volume();
    let error = measured / closed_form - 1.0;
    eprintln!(
        "{label}: signed volume {measured:.6} against {closed_form:.6} ({:+.4}%)",
        error * 100.0,
    );
    (error.abs() > BAND).then(|| {
        format!(
            "{label}: {measured} is {:+.4}% off the closed form {closed_form}",
            error * 100.0,
        )
    })
}

fn assert_signed_volume(label: &str, solid: &Solid, closed_form: f64) {
    if let Some(complaint) = signed_volume_row(label, solid, closed_form) {
        panic!("{complaint}");
    }
}

/// The five analytic in-repo fixtures against their own closed forms, SIGNED.
///
/// `occt-cube` measured -1000 and `occt-sphere` -523.253857 before spec 013 V1.
#[test]
fn step_loaded_shells_carry_their_closed_form_signed_volume() {
    const PI: f64 = std::f64::consts::PI;

    let complaints: Vec<String> = [
        ("occt-cube.step", 1000.0),
        ("occt-sphere.step", 4.0 / 3.0 * PI * 125.0),
        ("occt-cylinder.step", PI * 4.0 * 10.0),
        // Frustum: r = 2 at z = 0, half-angle atan(0.2) over h = 10, so R = 4.
        ("occt-cone.step", PI * 10.0 / 3.0 * (16.0 + 8.0 + 4.0)),
        ("occt-torus.step", 2.0 * PI * PI * 10.0 * 4.0),
    ]
    .into_iter()
    .filter_map(|(name, closed_form)| signed_volume_row(name, &extract(name), closed_form))
    .collect();

    assert!(
        complaints.is_empty(),
        "the divergence-theorem volume of {} loaded shell(s) is out of band:\n  {}\n\
         A value near the right MAGNITUDE with the wrong SIGN means the shell's faces \
         are oriented INWARD (ledger C15) -- this is the instrument \
         OperandVolume::trusted and verify_volume_conservation bound booleans with, and \
         neither can see it. Do NOT widen this band or take an abs(): it is 0.5% \
         against a 200% defect",
        complaints.len(),
        complaints.join("\n  "),
    );
}

/// Control arm: the same quantity on the kernel's OWN constructor, which never
/// touches the STEP load path. It measured `+1000` exactly throughout, and a
/// red row here would mean the volume ROUTINE moved rather than the load path.
#[test]
fn the_primitive_cuboid_control_is_exact_and_positive() {
    let cuboid = primitive::cuboid(BoundingBox::from_iter([
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(10.0, 10.0, 10.0),
    ]));
    assert_signed_volume("primitive::cuboid", &cuboid, 1000.0);
}

/// The FIRST of C15's two mechanisms, pinned where it happens rather than three
/// stages downstream at the volume.
///
/// ISO 10303-42 orients an `ADVANCED_FACE`'s bounding loops about the FACE
/// normal, which is the surface normal only when `same_sense` is `.T.`.
/// `CompressedFace`/`CompressedTrimmedFace` want the opposite convention:
/// `CompressDirector::create_cface` stores `face.boundaries` -- the ABSOLUTE
/// boundaries, in the surface's own sense -- next to the orientation flag, and
/// `create_face` rebuilds by `Face::try_new(stored, surface)` then `invert()`.
///
/// So every `same_sense = .F.` face whose loops are passed through verbatim is
/// traversed the wrong way round, and the shell is not closed. On
/// `occt-cube.step` that is exactly `#17`, `#237` and `#331` (against `#137`,
/// `#284`, `#338`, which are `.T.`), and it measured
/// `ShellCondition::Regular`: every one of the cube's twelve edges was
/// traversed the SAME way by both of its faces.
///
/// `normalize_trimmed_shell_orientation` then repaired that topologically --
/// it 2-coloured the shell, flood-filling from face 0 -- and, since face 0 is
/// `#17` (`.F.`), it flipped `#137`, `#284` and `#338` and landed the whole
/// cube INWARD: all six orientation flags `false`, volume `-1000`. Its doc says
/// it "does NOT decide global outwardness", and that is true and by design; the
/// defect is that it was handed an inconsistent shell at all.
#[test]
fn occt_cube_shell_is_consistently_oriented_as_loaded() {
    let csolid = compressed(&table("occt-cube.step"));
    let trimmed = monstertruck_topology::trimmed::TrimmedSolid::try_from(csolid)
        .expect("the cube compresses to a valid trimmed solid");
    for shell in trimmed.boundaries() {
        let plain: monstertruck_topology::Shell<_, _, _> = shell
            .faces()
            .iter()
            .map(|face| face.face().clone())
            .collect();
        assert_eq!(
            plain.shell_condition(),
            monstertruck_topology::shell::ShellCondition::Closed,
            "occt-cube's six faces must already agree on every shared edge as LOADED. \
             `Regular` here means the loader handed healing a shell whose faces \
             disagree, and orientation normalization then picks the global sign off \
             whichever face happens to be index 0 -- which is how a 1000-volume cube \
             came to measure -1000 (ledger C15)",
        );
    }
}
