//! The IGES path, end to end, over real files.
//!
//! # What these fixtures currently reach
//!
//! `cadmpeg-codec-iges` 0.6 decodes fixed-format ASCII IGES **4.0 through 5.3**,
//! so both fixtures here now reach the converter:
//!
//! | fixture | version | outcome (measured 2026-11-09, cadmpeg 0.6.11) |
//! | --- | --- | --- |
//! | `robot-rodney.igs` | 5.2 | 621 of 641 sheet bodies convert |
//! | `nist-ex1.iges` | 4.0 | decodes; its one wire body is refused by name |
//!
//! Under 0.5 the 4.0 file was refused by the decoder (cadmpeg/cadmpeg#133), and
//! the row pinning that refusal was written to fail when it lifted. It lifted.
//! `nist-ex1.iges` is an integrated-circuit drafting cell -- one wire body, no
//! surfaces -- so the correct answer for it is a named wire refusal and no faces.
//! It is a version specimen, not geometry.

#![cfg(feature = "iges")]

use monstertruck_io::Error;
use monstertruck_io::cadmpeg::ImportedBody;

fn fixture(name: &str) -> String {
    format!("{}/tests/fixtures/iges/{name}", env!("CARGO_MANIFEST_DIR"))
}

/// The claim that matters: a real 28 MB IGES 5.2 assembly reads, and its surfaces
/// arrive as monstertruck faces.
///
/// Counts are asserted with a floor rather than an equality. The exact number is
/// the decoder's to change between releases -- cadmpeg 0.4 grouped this same file
/// into 1266 bodies, 0.5 into 666 and 0.6 into 641 -- and pinning it here would
/// turn every upstream regrouping into a failure in this crate. What must not
/// change is that the file yields hundreds of faces rather than an error.
///
/// Every face of this file sits on an IGES 120 surface of revolution. The 0.6.11
/// IGES codec delivers a NURBS-generatrix revolution as `Solved(Nurbs)` and a
/// non-NURBS one as `Procedural { cache: None }`; it does not produce
/// `Procedural { cache: Some }` in non-test code. So the 621 sheets that convert
/// here are solved carriers, the 20 refused ones are procedural with no cache,
/// and the bridge's exact-cache arm is a policy for codecs that emit one rather
/// than the rescue this row was once said to prove.
#[test]
fn a_real_iges_5_2_assembly_reads_and_yields_faces() {
    let recovered = monstertruck_io::iges::from_path(fixture("robot-rodney.igs"))
        .expect("a 5.2 file must read");

    assert!(
        recovered.bodies.len() >= 600,
        "expected hundreds of bodies, got {} ({})",
        recovered.bodies.len(),
        recovered.summary()
    );

    let mut faces = 0;
    let mut sheets = 0;
    for body in &recovered.bodies {
        match body {
            ImportedBody::Sheet(sheet) => {
                sheets += 1;
                faces += sheet.faces.len();
            }
            ImportedBody::Solid(solid) => {
                faces += solid
                    .boundaries
                    .iter()
                    .map(|s| s.faces.len())
                    .sum::<usize>();
            }
        }
    }
    assert!(
        sheets >= 600,
        "IGES of this kind is sheet bodies; got {sheets}"
    );
    assert!(faces >= 600, "expected hundreds of faces, got {faces}");
}

/// The bodies that cannot convert are REFUSED BY NAME and do not cost us the
/// others.
///
/// This is the property the all-or-nothing version of the converter got wrong:
/// failing the call over a few unconvertible bodies returned an error for a file
/// that is overwhelmingly convertible.
#[test]
fn unconvertible_bodies_are_named_and_do_not_sink_the_file() {
    let recovered = monstertruck_io::iges::from_path(fixture("robot-rodney.igs"))
        .expect("a 5.2 file must read");

    assert!(
        !recovered.refused.is_empty(),
        "this fixture is known to carry unconvertible bodies; if that changed, this \
         row is no longer testing anything"
    );
    for refusal in &recovered.refused {
        assert!(
            !refusal.body.is_empty(),
            "a refusal must name the body it is about"
        );
        assert!(
            !refusal.reason.to_string().is_empty(),
            "a refusal must carry a reason"
        );
    }
    // Measured 2026-08-18 under 0.5: 641 of 666 converted; 24 sheets were refused
    // for composite curves and 1 wire body for having no equivalent. Measured
    // 2026-11-09 under 0.6.11: 621 of 641 convert; the 20 refused sheets carry a
    // surface of revolution the decoder delivered with NO solved cache. The point
    // is the RATIO, not the exact split -- refusals must be the tail, not the
    // outcome.
    assert!(
        recovered.bodies.len() > recovered.refused.len() * 10,
        "refusals should be a small tail: {}",
        recovered.summary()
    );
    assert!(recovered.summary().contains("refused"));
}

/// The strict entry point still exists and still refuses this file.
///
/// Keeping it honest: `all_bodies_from_path` is the all-or-nothing reading, and on
/// real IGES it fails. That is why it is not the default.
#[test]
fn the_strict_entry_point_refuses_a_file_that_carries_wires() {
    let error = monstertruck_io::iges::all_bodies_from_path(fixture("robot-rodney.igs"))
        .expect_err("the strict reading must refuse a file with unconvertible bodies");
    // Which refusal comes first is traversal order, not a contract. Under 0.5 it
    // was a composite curve; under 0.6 every refusal on this file is a procedural
    // surface with no solved cache.
    assert!(
        matches!(
            error,
            Error::UnsupportedBodyKind { .. }
                | Error::UnsupportedCurveKind { .. }
                | Error::UnsupportedSurfaceKind { .. }
        ),
        "got {error}"
    );
}

/// IGES 4.0 is inside cadmpeg 0.6's envelope: the file decodes, and its one
/// wire body is refused by name, with no faces. That is the correct answer for
/// an IC drafting cell, and it replaces the 0.5 pin on the decoder's refusal.
#[test]
fn an_iges_4_0_file_decodes_and_its_wire_body_is_refused_by_name() {
    let recovered = monstertruck_io::iges::from_path(fixture("nist-ex1.iges"))
        .expect("cadmpeg 0.6 reads IGES 4.0");
    assert!(
        recovered.bodies.is_empty(),
        "a drafting cell has no faces to convert: {}",
        recovered.summary()
    );
    assert_eq!(recovered.refused.len(), 1, "{}", recovered.summary());
    let refusal = &recovered.refused[0];
    assert_eq!(refusal.kind, "wire");
    assert!(
        matches!(
            refusal.reason,
            Error::UnsupportedBodyKind { kind: "wire", .. }
        ),
        "got {}",
        refusal.reason
    );
}

/// The decoder's own loss report reaches the caller on a decode that SUCCEEDED.
///
/// Before the 0.6 port, `Import` carried only what the converter refused; what
/// the decoder itself dropped stopped at the decoder boundary. Measured on this
/// fixture: 8 decoder losses, among them entities the decoder declined to project
/// (`iges/entity.not-projected`). Asserted as presence and shape, not as a count,
/// which is the decoder's to change.
#[test]
fn the_decoders_losses_reach_the_caller_on_a_successful_decode() {
    let recovered = monstertruck_io::iges::from_path(fixture("nist-ex1.iges"))
        .expect("cadmpeg 0.6 reads IGES 4.0");
    assert!(
        !recovered.losses.is_empty(),
        "this fixture is known to carry decoder losses; if that changed, this row is no \
         longer testing anything"
    );
    for loss in &recovered.losses {
        assert!(
            loss.code.starts_with("iges/"),
            "an IGES loss code: {}",
            loss.code
        );
        assert!(!loss.message.is_empty(), "a loss must say what was lost");
    }
    assert!(
        recovered
            .losses
            .iter()
            .any(|loss| loss.code == "iges/entity.not-projected"),
        "{:?}",
        recovered.losses
    );
    assert!(
        recovered.summary().contains("losses"),
        "{}",
        recovered.summary()
    );
}

/// A converted sheet is real topology, not just a count.
#[test]
fn a_converted_sheet_extracts_as_a_shell() {
    let recovered = monstertruck_io::iges::from_path(fixture("robot-rodney.igs"))
        .expect("a 5.2 file must read");
    let sheet = recovered
        .bodies
        .iter()
        .find_map(|body| match body {
            ImportedBody::Sheet(sheet) if !sheet.faces.is_empty() => Some(sheet),
            _ => None,
        })
        .expect("at least one sheet with a face");

    // HEALED extraction, not the bare one. Every sheet in this file carries a
    // closed edge -- an edge whose start and end vertex are the same -- and
    // `Edge::try_new` rejects those with `SameVertex`. That is not a defect in the
    // conversion: it is what the file contains, and `Shell::extract`'s own
    // documentation points at `SplitClosedEdgesAndFaces` for exactly this.
    //
    // Measured: bare extraction fails on all 641 converted sheets; healed
    // extraction succeeds on all 641.
    let shell = monstertruck_healing::extract_healed(sheet.clone(), 0.01)
        .expect("a converted sheet must extract once its closed edges are split");
    assert_eq!(shell.len(), sheet.faces.len());
}

/// The fixtures are present and are not unfetched Git LFS pointers.
///
/// Without this, a checkout missing LFS objects fails every row above with a
/// confusing decode error instead of the real cause.
#[test]
fn the_fixtures_are_real_files_and_not_lfs_pointers() {
    for (name, least) in [("nist-ex1.iges", 4_000), ("robot-rodney.igs", 1_000_000)] {
        let path = fixture(name);
        let size = std::fs::metadata(&path)
            .unwrap_or_else(|error| panic!("{name} is missing: {error}"))
            .len();
        assert!(
            size >= least,
            "{name} is {size} bytes, too small to be the fixture -- almost certainly \
             an unfetched Git LFS pointer. Run `git lfs pull`."
        );
    }
}

/// An explicit ceiling actually refuses, and refuses as a typed error.
///
/// Without this the knob is an untested arm: `from_path_with_policy` would compile,
/// be documented, and never have been shown to do anything. The ceiling here is
/// absurdly low on purpose -- 100 entities against a file holding tens of thousands
/// -- so the refusal is about the limit and not about the file.
///
/// Note what is asserted: `Error::Decode`, i.e. the DECODER refused. That is the
/// point of the knob -- a catchable failure instead of a long grind.
#[test]
fn an_explicit_resource_ceiling_refuses_a_file_that_exceeds_it() {
    use monstertruck_io::iges::{DecodePolicy, ResourceLimits};

    let mut limits = ResourceLimits::service();
    limits.max_entities = 100;
    let policy = DecodePolicy {
        limits,
        ..DecodePolicy::service()
    };

    let error = monstertruck_io::iges::from_path_with_policy(fixture("robot-rodney.igs"), policy)
        .expect_err("a 100-entity ceiling must refuse a file with tens of thousands");
    assert!(
        matches!(error, Error::Decode { format: "IGES", .. }),
        "the ceiling must surface as a decoder refusal; got {error}"
    );
}

/// The default policy still reads the file, so the knob did not change behaviour
/// for callers who do not pass one.
#[test]
fn the_default_policy_still_reads_the_fixture() {
    let recovered = monstertruck_io::iges::from_path(fixture("robot-rodney.igs"))
        .expect("the default ceiling must not refuse this file");
    assert!(recovered.bodies.len() >= 600, "{}", recovered.summary());
}
