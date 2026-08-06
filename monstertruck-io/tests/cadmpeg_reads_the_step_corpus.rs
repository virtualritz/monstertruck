//! The cadmpeg-backed STEP reader must read the same corpus ours does.
//!
//! `monstertruck-io` carries two STEP readers on purpose: our own on `step-p21`
//! (the default, and the measurement baseline the boolean kernel is
//! characterised against) and cadmpeg's behind the `cadmpeg` feature. Keeping
//! the second one compiled but unexercised would tell us nothing about whether
//! it could ever replace the first, which is the only reason it is here.
//!
//! So this row reads every in-repo STEP fixture through cadmpeg and asserts the
//! recovered *topology*, not merely that decoding returned `Ok`. A decoder that
//! parses a file and recovers no faces has not read it in any sense that matters
//! to a B-rep kernel -- and that was the real failure mode: on 2026-08-04
//! `occt-cylinder.step` decoded "successfully" while dropping its entire
//! `MANIFOLD_SOLID_BREP`, because one parameter-space `CIRCLE` failed
//! (cadmpeg/cadmpeg#79, fixed in cadmpeg/cadmpeg#83).
//!
//! The expected counts below are MEASURED against cadmpeg 0.4, not guessed, and
//! they are exact: STEP decoding is deterministic, so a change in either the
//! fixture or the decoder trips this row rather than passing quietly.

#![cfg(feature = "cadmpeg")]

use monstertruck_io::cadmpeg::step::decode_file_to_ir;
use std::path::{Path, PathBuf};

/// `(fixture, bodies, faces, edges, vertices)`, measured against cadmpeg 0.4.
///
/// `occt-cube` is the one to sanity-check by hand: 6 faces, 12 edges, 8 vertices
/// gives Euler 8 - 12 + 6 = 2, a closed genus-0 solid. A sphere and a torus are
/// single seam-degenerate faces, hence zero edges.
const EXPECTED: [(&str, usize, usize, usize, usize); 10] = [
    ("abc-0000.step", 6, 25, 33, 32),
    ("abc-0006.step", 1, 15, 32, 20),
    ("abc-0008.step", 1, 21, 34, 26),
    ("abc-0035.step", 1, 82, 188, 122),
    ("occt-assy.step", 2, 9, 15, 10),
    ("occt-cone.step", 1, 3, 3, 2),
    ("occt-cube.step", 1, 6, 12, 8),
    ("occt-cylinder.step", 1, 3, 3, 2),
    ("occt-sphere.step", 1, 1, 0, 1),
    ("occt-torus.step", 1, 1, 0, 1),
];

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../resources/step")
        .join(name)
}

#[test]
fn cadmpeg_recovers_the_expected_topology_from_every_fixture() {
    let mut failures = Vec::new();

    for (name, bodies, faces, edges, vertices) in EXPECTED {
        let path = fixture(name);
        assert!(
            path.is_file(),
            "fixture {} is missing -- it moved, or a rename left this path behind",
            path.display(),
        );

        let ir = match decode_file_to_ir(&path) {
            Ok(ir) => ir,
            Err(error) => {
                failures.push(format!("{name}: decode failed: {error}"));
                continue;
            }
        };

        let got = (
            ir.model.bodies.len(),
            ir.model.faces.len(),
            ir.model.edges.len(),
            ir.model.vertices.len(),
        );
        if got != (bodies, faces, edges, vertices) {
            failures.push(format!(
                "{name}: bodies/faces/edges/vertices = {got:?}, expected \
                 {:?}",
                (bodies, faces, edges, vertices)
            ));
        }
    }

    assert!(
        failures.is_empty(),
        "{} of {} fixtures did not match:\n  {}",
        failures.len(),
        EXPECTED.len(),
        failures.join("\n  "),
    );
}

/// The fixture that regressed, pinned on its own so a failure names the defect.
#[test]
fn the_cylinder_that_once_lost_its_whole_solid_now_keeps_it() {
    let ir = decode_file_to_ir(fixture("occt-cylinder.step")).expect("decode occt-cylinder");

    // Not "did it parse" -- did the MANIFOLD_SOLID_BREP survive. A cylinder is
    // three faces: the lateral surface and two caps.
    assert_eq!(
        ir.model.faces.len(),
        3,
        "the solid collapsed again: cadmpeg/cadmpeg#79 was one parameter-space \
         CIRCLE taking the entire MANIFOLD_SOLID_BREP with it"
    );
    assert_eq!(ir.model.bodies.len(), 1, "expected one body");
}
