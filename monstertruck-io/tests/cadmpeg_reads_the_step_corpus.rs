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
//! The expected counts below are MEASURED, not guessed, and they are exact:
//! STEP decoding is deterministic, so a change in either the fixture or the
//! decoder trips this row rather than passing quietly.
//!
//! # 2026-09-09: nine of these ten fixtures REGRESSED between 0.4 and 0.5.5
//!
//! At 0.5.5 only `occt-assy` still decoded; the other nine were refused before
//! any geometry was reached, by container invariants (`HEADER must begin with
//! FILE_DESCRIPTION, FILE_NAME, and FILE_SCHEMA` for the five `occt-*`
//! fixtures, `FILE_DESCRIPTION has ...` for the four `abc-*` ones). The table
//! pinned that regression so that it would fire the moment cadmpeg accepted the
//! files again.
//!
//! # 2026-11-09: at 0.6.11 all ten decode again
//!
//! It fired. 0.6 reads the non-conforming headers by name and REPORTS them as
//! decoder losses (`step/parse.header-metadata-noncanonical`,
//! `step/parse.implementation-level-unverified`) instead of refusing the file,
//! which is the importer behaviour our own reader always had. The counts below
//! are the 0.6.11 measurement; `occt-assy` is unchanged from 0.4.
//!
//! The wider comparison against our own reader, over the real-world corpus this
//! file does not touch, lives in
//! `cadmpeg_versus_ours_on_the_real_corpus.rs`.

#![cfg(feature = "cadmpeg")]

use monstertruck_io::cadmpeg::step::{decode_file_to_ir, import_from_path};
use std::path::{Path, PathBuf};

/// Every fixture, with `(bodies, faces, edges, vertices)` as cadmpeg decodes it.
const DECODES: [(&str, usize, usize, usize, usize); 10] = [
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

/// One refusal word and the number of instances it stands for.
type RefusalCensus = [(&'static str, usize)];

/// `(fixture, converted bodies, refusal census)`.
type ConversionOutcome = (&'static str, usize, &'static RefusalCensus);

/// Every fixture's CONVERSION outcome: `(converted bodies, sorted refusal words
/// with instance counts)`. Measured on this branch; a converter that refused
/// every `abc` body, or silently converted fewer, now fails here rather than
/// passing because it merely attempted the conversion.
const CONVERTS: [ConversionOutcome; 10] = [
    (
        "abc-0000.step",
        1,
        &[
            ("malformed geometry", 1),
            ("singular (pole) face boundary", 4),
        ],
    ),
    ("abc-0006.step", 1, &[]),
    ("abc-0008.step", 0, &[("malformed geometry", 1)]),
    ("abc-0035.step", 0, &[("malformed geometry", 1)]),
    ("occt-assy.step", 5, &[]),
    ("occt-cone.step", 1, &[]),
    ("occt-cube.step", 1, &[]),
    ("occt-cylinder.step", 1, &[]),
    (
        "occt-sphere.step",
        0,
        &[("singular (pole) face boundary", 1)],
    ),
    (
        "occt-torus.step",
        0,
        &[("singular (pole) face boundary", 1)],
    ),
];

/// The stable refusal word, without the entity id that `Display` appends.
fn refusal_word(reason: &monstertruck_io::Error) -> &'static str {
    use monstertruck_io::Error;
    match reason {
        Error::UnsupportedSurfaceKind { kind, .. }
        | Error::UnsupportedCurveKind { kind, .. }
        | Error::UnsupportedBodyKind { kind, .. }
        | Error::UnplaceableOccurrence { kind, .. } => kind,
        Error::MalformedGeometry { .. } => "malformed geometry",
        _ => "other",
    }
}

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../resources/step")
        .join(name)
}

#[test]
fn cadmpeg_recovers_the_expected_topology_from_every_fixture() {
    let mut failures = Vec::new();

    for (name, bodies, faces, edges, vertices) in DECODES {
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

        // Decoding is not converting. Every fixture must reach the converter,
        // and every body it refuses must be NAMED: an `Ok` with no body and no
        // refusal is the silent-empty result this reader exists to avoid.
        match monstertruck_io::cadmpeg::import(&ir, "STEP") {
            Err(error) => failures.push(format!("{name}: conversion failed: {error}")),
            Ok(recovered) => {
                if recovered.bodies.is_empty() && recovered.refused.is_empty() {
                    failures.push(format!("{name}: converted to nothing without a refusal"));
                }
                let expected = CONVERTS
                    .iter()
                    .find(|(fixture, ..)| *fixture == name)
                    .expect("every decoded fixture has a pinned conversion outcome");
                // Count INSTANCES per refusal word, so one entry per refused body
                // and one entry per whole uninstanced body compare the same way.
                let mut refused: std::collections::BTreeMap<&str, usize> = Default::default();
                for refusal in &recovered.refused {
                    *refused.entry(refusal_word(&refusal.reason)).or_default() += refusal.instances;
                }
                let mut expected_refused: std::collections::BTreeMap<&str, usize> =
                    Default::default();
                for &(word, instances) in expected.2 {
                    *expected_refused.entry(word).or_default() += instances;
                }
                if recovered.bodies.len() != expected.1 || refused != expected_refused {
                    failures.push(format!(
                        "{name}: converted {} bodies with {refused:?}, expected {} with {expected_refused:?}",
                        recovered.bodies.len(),
                        expected.1,
                    ));
                }
                let accounted: usize = recovered.bodies.len()
                    + recovered
                        .refused
                        .iter()
                        .map(|refusal| refusal.instances)
                        .sum::<usize>();
                if accounted != recovered.total() {
                    failures.push(format!(
                        "{name}: total() = {} but bodies + refused instances = {accounted}",
                        recovered.total()
                    ));
                }
            }
        }
    }

    assert!(
        failures.is_empty(),
        "{} of {} fixtures did not match:\n  {}",
        failures.len(),
        DECODES.len(),
        failures.join("\n  "),
    );
}

/// The fixture that regressed twice, pinned on its own so a failure names which.
///
/// At 0.4 this file decoded but silently dropped its whole `MANIFOLD_SOLID_BREP`
/// when one parameter-space `CIRCLE` failed (cadmpeg/cadmpeg#79, fixed in #83).
/// At 0.5.5 it was refused outright, because its FreeCAD-written HEADER omits
/// `FILE_NAME`. At 0.6.11 it decodes, reports the header as a loss, and
/// CONVERTS: the three faces our own reader recovers, in one solid.
#[test]
fn the_cylinder_decodes_converts_and_reports_its_header() {
    let recovered = import_from_path(fixture("occt-cylinder.step"))
        .expect("0.6 reads this file; if it is refused again, the header check is back");
    assert!(recovered.refused.is_empty(), "{}", recovered.summary());
    assert_eq!(recovered.bodies.len(), 1);
    let monstertruck_io::cadmpeg::ImportedBody::Solid(solid) = &recovered.bodies[0] else {
        panic!("a solid body must convert to a solid");
    };
    let faces: usize = solid.boundaries.iter().map(|shell| shell.faces.len()).sum();
    assert_eq!(faces, 3, "a cylinder is two caps and one side");
    assert!(
        recovered
            .losses
            .iter()
            .any(|loss| loss.code == "step/parse.header-metadata-noncanonical"),
        "the missing FILE_NAME must reach the caller as a decoder loss: {:?}",
        recovered.losses
    );
}

/// `occt-assy` is a roof and FOUR instances of one tire definition. The tire's
/// body exists once in the decoded document; each placed instance must come
/// back as its own body at its own placement. Under 0.5 the bridge returned two
/// bodies -- the tire once, at its definition's origin.
#[test]
fn every_instance_of_an_assembly_part_converts_at_its_own_placement() {
    use monstertruck_io::cadmpeg::ImportedBody;
    let recovered = import_from_path(fixture("occt-assy.step")).expect("the assembly reads");
    assert!(recovered.refused.is_empty(), "{}", recovered.summary());
    assert_eq!(
        recovered.bodies.len(),
        5,
        "one roof and four tires: {}",
        recovered.summary()
    );
    let shapes: Vec<(usize, [f64; 3])> = recovered
        .bodies
        .iter()
        .map(|body| {
            let ImportedBody::Solid(solid) = body else {
                panic!("every body here is a solid");
            };
            let shell = &solid.boundaries[0];
            let count = shell.vertices.len() as f64;
            let mut centre = [0.0; 3];
            for vertex in &shell.vertices {
                centre[0] += vertex.x / count;
                centre[1] += vertex.y / count;
                centre[2] += vertex.z / count;
            }
            (shell.faces.len(), centre)
        })
        .collect();
    // Four bodies share the tire's topology; they must sit at four DIFFERENT
    // places, or the instancing collapsed them.
    let tire_faces = shapes
        .iter()
        .map(|(faces, _)| *faces)
        .find(|faces| shapes.iter().filter(|(other, _)| other == faces).count() == 4)
        .unwrap_or_else(|| panic!("four congruent tires: {shapes:?}"));
    let tires: Vec<[f64; 3]> = shapes
        .iter()
        .filter(|(faces, _)| *faces == tire_faces)
        .map(|(_, centre)| *centre)
        .collect();
    for (index, a) in tires.iter().enumerate() {
        for b in &tires[index + 1..] {
            let gap =
                ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt();
            assert!(gap > 1.0e-6, "two tire instances coincide: {tires:?}");
        }
    }
}
