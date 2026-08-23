//! An importer must never return an empty model where it means something else.
//!
//! The failure this guards against is specific: a caller writes
//! `let bodies = iges::from_path(p)?;`, gets `Ok(vec![])`, and concludes the file
//! held no geometry. That reading is wrong and silent, and it stays wrong until
//! someone compares against another tool.
//!
//! # This file used to assert the opposite
//!
//! Until the converter was written it asserted `Error::Unimplemented`, and that
//! row went **red on master** the day the conversion landed -- unnoticed, because
//! it is `#![cfg(feature = "iges")]` and the default-feature suite never compiles
//! it. Its failure message was the good news: *"returned Ok with 1 bodies"*.
//!
//! It is kept, rather than deleted, because the invariant it protects is not
//! about being unimplemented. The two ways to report "nothing" still have to stay
//! distinguishable from each other and from success, and now there is a third
//! thing to hold: that a real document converts.

#![cfg(feature = "iges")]

use cadmpeg_ir::{CadIr, examples, units::Units};
use monstertruck_io::cadmpeg::{ImportedBody, to_bodies};
use monstertruck_io::Error;
use monstertruck_topology::{Shell, shell::ShellCondition};

/// A document with no bodies is a fact about the file, so it keeps its own error.
///
/// This must not collapse into `Ok(vec![])` now that the conversion works: an
/// empty file and a converted-to-nothing file would then be indistinguishable.
#[test]
fn an_empty_document_reports_no_geometry_rather_than_an_empty_list() {
    let ir = CadIr::empty(Units::default());
    match to_bodies(&ir, "IGES") {
        Err(Error::NoGeometry { format }) => assert_eq!(format, "IGES"),
        other => panic!("expected NoGeometry, got {other:?}"),
    }
}

/// The claim the old row was waiting for: a real intermediate representation
/// converts into a real solid.
///
/// This is stronger evidence than the converter's own unit tests, which build a
/// `CadIr` by hand and could agree with a shared misreading. `examples::unit_cube`
/// is **cadmpeg's** document, written by the people who define the format, so it
/// exercises the graph a decoded file actually produces.
///
/// Every number here is checked, not just the body count. A cube is 8 vertices,
/// 12 edges and 6 faces -- Euler characteristic 2 -- and `Shell::extract` running
/// `Edge::try_new` and `Face::try_new` over the result reports it CLOSED. A
/// converter that dropped an edge, or oriented one backwards, gets a body count of
/// 1 and fails every line below it.
#[test]
fn cadmpegs_own_unit_cube_converts_to_a_closed_solid() {
    let ir = examples::unit_cube();
    assert!(!ir.model.bodies.is_empty(), "the example must carry a body");

    let bodies = to_bodies(&ir, "IGES").expect("cadmpeg's unit cube must convert");
    assert_eq!(bodies.len(), 1, "one body in, one body out");

    let ImportedBody::Solid(solid) = &bodies[0] else {
        panic!("a solid body must convert to a solid, got {:?}", bodies[0]);
    };
    assert_eq!(solid.boundaries.len(), 1, "a cube has one boundary shell");

    let shell = &solid.boundaries[0];
    assert_eq!(shell.vertices.len(), 8, "a cube has 8 vertices");
    assert_eq!(shell.edges.len(), 12, "a cube has 12 edges, each shared by two faces");
    assert_eq!(shell.faces.len(), 6, "a cube has 6 faces");
    // V - E + F = 2. Stated separately: the three counts above could each be
    // wrong in a way that still looks plausible, and this is the relation that
    // says they describe a sphere-like surface rather than three numbers.
    assert_eq!(
        shell.vertices.len() + shell.faces.len() - shell.edges.len(),
        2,
        "Euler characteristic must be 2"
    );

    let extracted = Shell::extract(shell.clone()).expect("the shell must extract");
    assert_eq!(extracted.len(), 6);
    assert_eq!(
        extracted.shell_condition(),
        ShellCondition::Closed,
        "the converted cube must bound a volume; anything less means an edge is \
         unshared or a boundary does not close"
    );
}

/// Success must remain distinguishable from "nothing found".
///
/// `to_bodies` returning `Ok(vec![])` should be unreachable -- an empty document
/// is `NoGeometry`, and a body that cannot convert is a typed refusal naming
/// itself. This pins that there is no third path that quietly yields an empty
/// list, which is the shape the whole converter was built to avoid.
#[test]
fn no_input_produces_an_empty_ok() {
    for (name, ir) in [
        ("empty", CadIr::empty(Units::default())),
        ("unit cube", examples::unit_cube()),
    ] {
        if let Ok(bodies) = to_bodies(&ir, "IGES") {
            assert!(
                !bodies.is_empty(),
                "{name} returned Ok with no bodies -- success and \"found nothing\" must not \
                 look the same"
            );
        }
    }
}
