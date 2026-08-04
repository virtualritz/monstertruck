//! A placeholder importer must REFUSE, not return an empty model.
//!
//! The failure this guards against is specific: a caller writes
//! `let solids = iges::from_path(p)?;`, gets `Ok(vec![])`, and concludes the file
//! held no geometry. That reading is wrong and silent, and it stays wrong until
//! someone compares against another tool. So while the conversion is unwritten,
//! every path returns a typed error, and this row fails the day someone
//! "helpfully" makes it return an empty vector instead.

#![cfg(feature = "iges")]

use cadmpeg_ir::{CadIr, examples, units::Units};
use monstertruck_io::{Error, cadmpeg::to_solids};

/// A document with no bodies is a fact about the file, so it gets its own error
/// rather than the not-implemented one -- otherwise finishing the converter
/// would silently change what an empty file means.
#[test]
fn an_empty_document_reports_no_geometry_not_unimplemented() {
    let ir = CadIr::empty(Units::default());
    match to_solids(&ir, "IGES") {
        Err(Error::NoGeometry { format }) => assert_eq!(format, "IGES"),
        other => panic!("expected NoGeometry, got {other:?}"),
    }
}

/// The unwritten conversion must never look like success.
#[test]
fn the_unwritten_conversion_refuses() {
    // A real IR document with actual topology, from cadmpeg's own examples, so
    // the row exercises the path a decoded file takes rather than a stub.
    let ir = examples::unit_cube();
    assert!(!ir.model.bodies.is_empty(), "the example must carry a body");
    match to_solids(&ir, "IGES") {
        Err(Error::Unimplemented { what }) => {
            assert!(!what.is_empty(), "the error must name what is missing");
        }
        Ok(solids) => panic!(
            "returned Ok with {} solids -- a placeholder that reports success is \
             indistinguishable from a working importer that found nothing",
            solids.len()
        ),
        other => panic!("expected Unimplemented, got {other:?}"),
    }
}
