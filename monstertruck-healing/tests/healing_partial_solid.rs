//! Ledger class C11 -- an infallible constructor on a path real data violates.
//!
//! # What this file pins
//!
//! The STEP loader converts a shell FACE BY FACE, so a typed surface refusal
//! (spec 011 T1's degenerate-torus refusal, for instance) drops that one face
//! and the shell is still returned as `Ok`. What happened next, measured on
//! `ROTOR-201NAL-Z7.STEP`, was NOT a refusal:
//!
//! * `extract_healed_trimmed_solid` returned `Ok` on the two-face-short shell,
//!   because it ended in the UNCHECKED `TrimmedSolid::new`;
//! * `TrimmedSolid::erase_trims` built the plain `Solid` through
//!   `Solid::new_unchecked`, so nothing complained there either;
//! * `Solid::try_mapped` then went through `Solid::debug_new`, which under
//!   `debug_assertions` WAS `Solid::new` -- `try_new(..).unwrap_or_else(|e|
//!   panic!(..))` -- and the process ABORTED. In a release build the same call
//!   was `new_unchecked`, so it would instead have returned `Some(invalid
//!   solid)` and let a boolean run on a shell with a hole. Spec 012 U4 removed
//!   that profile switch: `Solid::debug_new` returns `Result` and
//!   `try_mapped` answers `None`. The healing-stage refusal below is still the
//!   fix -- U4 only removes the second, profile-dependent outcome behind it.
//!
//! Three of ROTOR's 33 solids were in that state. The fix is
//! `TrimmedSolid::try_new` at the end of the healing path: the last stage that
//! can still repair anything is also the last one that can honestly refuse.
//!
//! The rows below run on an in-repo fixture, so they are part of the default
//! gate; the corpus row that pins the same thing on real ROTOR geometry lives
//! with the external SSI boolean backend.

use monstertruck_io::step::load::{
    Table,
    step_geometry::{Curve3D, StepParameterCurve, Surface as StepSurface},
};
use monstertruck_modeling::*;
use monstertruck_topology::compress::CompressedTrimmedSolid;
use monstertruck_topology::errors::Error;

const TOL: f64 = 1.0e-3;

type StepTrimmedSolid = CompressedTrimmedSolid<Point3, Curve3D, StepSurface, StepParameterCurve>;

/// The smallest in-repo closed solid: a six-face cube.
fn cube() -> StepTrimmedSolid {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../resources/step/occt-cube.step"
    );
    let bytes = std::fs::read(path).expect("occt-cube.step is an in-repo fixture");
    let table = Table::from_step_bytes(&bytes).expect("occt-cube.step must table-parse");
    let (_, holder) = table
        .manifold_solid_brep
        .iter()
        .next()
        .expect("occt-cube.step has one MANIFOLD_SOLID_BREP");
    table
        .to_compressed_trimmed_solid(holder)
        .expect("the cube must compress")
}

/// Control arm. A complete solid must still extract to `Ok` -- the refusal
/// added for the partial case must not cost a single good input. (Measured
/// over all 15 in-repo STEP fixtures, 185 solids: zero moved.)
#[test]
fn a_complete_compressed_solid_still_heals_and_extracts() {
    let csolid = cube();
    let faces: usize = csolid.boundaries.iter().map(|s| s.faces.len()).sum();
    assert_eq!(faces, 6, "the fixture must be the six-face cube");

    let solid = monstertruck_healing::extract_healed_trimmed_solid(csolid, TOL)
        .expect("a complete cube must extract");
    assert_eq!(solid.boundaries().len(), 1);
    assert_eq!(solid.boundaries()[0].faces().len(), 6);
}

/// The C11 row, inverted. A compressed solid one face short -- exactly the
/// shape a typed surface refusal upstream leaves behind -- must produce a
/// TYPED refusal from the healing path, and must reach neither the abort in
/// `Solid::debug_new` nor an `Ok` that carries a solid with a hole in it.
#[test]
fn a_compressed_solid_that_lost_a_face_refuses_typed_instead_of_aborting() {
    let mut csolid = cube();
    let dropped = csolid.boundaries[0].faces.pop().expect("six faces");
    assert_eq!(
        csolid.boundaries[0].faces.len(),
        5,
        "the input must be one face short, with its edges and vertices intact -- \
         a dropped face, not a truncated shell",
    );
    drop(dropped);

    // The whole recipe the callers use, inside `catch_unwind`: before the fix
    // this UNWOUND at `monstertruck-topology/src/solid.rs`'s `Solid::new`, so
    // asserting on the returned `Err` alone would not have witnessed anything.
    let outcome = std::panic::catch_unwind(move || {
        monstertruck_healing::extract_healed_trimmed_solid(csolid, TOL).map(|solid| {
            solid
                .erase_trims()
                .try_mapped(
                    |point| Some(*point),
                    |curve: &Curve3D| Curve::try_from(curve).ok(),
                    |surface: &StepSurface| Surface::try_from(surface).ok(),
                )
                .is_some()
        })
    });

    let result = outcome.expect(
        "extraction of a face-short solid must not PANIC; an unwind here means the \
         infallible constructor is back on this path (ledger class C11)",
    );
    assert_eq!(
        result.err(),
        Some(Error::NotClosedShell),
        "a shell that lost a face is not closed, and the healing path must say so \
         with the same typed error the plain `Solid::extract` path has always used",
    );
}

/// The refusal must name WHICH invariant broke. "Something is wrong with this
/// solid" is what the panic already said; a caller deciding whether to retry,
/// split or reject needs the distinction, and a single collapsed error variant
/// would quietly lose it.
#[test]
fn the_refusal_distinguishes_an_open_shell_from_a_disconnected_one() {
    let mut open = cube();
    open.boundaries[0].faces.pop();
    assert_eq!(
        monstertruck_healing::extract_healed_trimmed_solid(open, TOL).err(),
        Some(Error::NotClosedShell),
    );

    // Two whole cubes presented as ONE boundary shell. Every face is closed;
    // what is wrong is that this is two solids, not one.
    let mut disconnected = cube();
    let second = cube();
    let vertex_offset = disconnected.boundaries[0].vertices.len();
    let edge_offset = disconnected.boundaries[0].edges.len();
    disconnected.boundaries[0]
        .vertices
        .extend(second.boundaries[0].vertices.iter().cloned());
    for mut edge in second.boundaries[0].edges.iter().cloned() {
        edge.vertices = (
            edge.vertices.0 + vertex_offset,
            edge.vertices.1 + vertex_offset,
        );
        disconnected.boundaries[0].edges.push(edge);
    }
    for mut face in second.boundaries[0].faces.iter().cloned() {
        for wire in &mut face.boundaries {
            for edge_use in wire.iter_mut() {
                edge_use.index += edge_offset;
            }
        }
        disconnected.boundaries[0].faces.push(face);
    }

    assert_eq!(
        monstertruck_healing::extract_healed_trimmed_solid(disconnected, TOL).err(),
        Some(Error::NotConnected),
        "two disjoint closed shells in one boundary must refuse as NotConnected, \
         not as NotClosedShell",
    );
}
