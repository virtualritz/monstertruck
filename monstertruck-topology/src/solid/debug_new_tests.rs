//! Unit tests for the parent module (`debug_new_tests`).
//!
//! Split out of the module file so the source stays readable. The module
//! name is unchanged, so every test keeps its path and its identity.

use crate::*;

/// An open shell -- one triangle. Not a valid solid boundary (not closed),
/// which is the shape a shell that lost a face upstream arrives in.
fn open_shell() -> Shell<(), (), ()> {
    let v = Vertex::from_points([(); 3]);
    let wire: Wire<(), ()> = vec![
        Edge::new(&v[0], &v[1], ()),
        Edge::new(&v[1], &v[2], ()),
        Edge::new(&v[2], &v[0], ()),
    ]
    .into();
    std::iter::once(Face::new(vec![wire], ())).collect()
}

/// The constructor REPORTS instead of aborting. Under `debug_assertions`
/// that is a typed `Err`; in release the check is skipped by design, so the
/// row pins the profile it is running in rather than pretending there is
/// only one.
#[test]
fn debug_new_reports_an_open_shell_instead_of_panicking() {
    let outcome = Solid::debug_new(vec![open_shell()]);
    if cfg!(debug_assertions) {
        assert!(
            matches!(outcome, Err(errors::Error::NotClosedShell)),
            "an open shell must come back as a typed refusal, not a panic",
        );
    } else {
        assert!(
            outcome.is_ok(),
            "release still skips the check -- `debug_new` is what it is called",
        );
    }
}

/// The measured C11 instance, at topology level: a solid carrying an
/// invalid shell (only reachable through `new_unchecked`, which is how
/// `TrimmedSolid::erase_trims` builds one) used to ABORT the process here
/// in debug and hand back `Some(invalid solid)` in release. `try_mapped`
/// has an `Option` and now uses it.
///
/// The release arm is not a weaker assertion of the same thing -- it is the
/// C9 face of the class, and it is asserted here precisely because a
/// panic-shaped census cannot see it.
#[test]
fn try_mapped_over_an_invalid_solid_refuses_instead_of_aborting() {
    let invalid = Solid::new_unchecked(vec![open_shell()]);
    let mapped = invalid.try_mapped(|_| Some(()), |_| Some(()), |_| Some(()));
    if cfg!(debug_assertions) {
        assert!(
            mapped.is_none(),
            "the debug face of C11: an abort, now a `None` this signature \
             already promised",
        );
    } else {
        assert!(
            mapped.is_some(),
            "the release face of C11 is C9 -- documented, not fixed by a \
             fallible signature alone",
        );
    }
}

/// A VALID solid must be unaffected: the refusal added above costs nothing
/// on any input that was already good, in either profile.
#[test]
fn try_mapped_over_a_valid_solid_is_unmoved() {
    let mapped = super::cube().try_mapped(|_| Some(()), |_| Some(()), |_| Some(()));
    let mapped = mapped.expect("a closed cube maps");
    assert_eq!(mapped.boundaries().len(), 1);
    assert_eq!(mapped.boundaries()[0].len(), 6);
}

/// `mapped` is infallible by signature, so it cannot report -- and it must
/// not abort either. It is a total structure-preserving map: the result is
/// exactly as valid as the receiver, which is the only honest thing an
/// infallible mapping can promise.
#[test]
fn mapped_over_an_invalid_solid_neither_aborts_nor_launders() {
    let invalid = Solid::new_unchecked(vec![open_shell()]);
    let mapped = invalid.mapped(|_| (), |_| (), |_| ());
    assert_eq!(mapped.boundaries().len(), 1);
    assert!(
        matches!(
            mapped.boundaries()[0].check_solid_boundary(),
            Err(errors::Error::NotClosedShell)
        ),
        "the receiver's invalidity is preserved, not repaired and not hidden",
    );
}
