//! Unit tests for the parent module (`debug_new_tests`).
//!
//! Split out of the module file so the source stays readable. The module
//! name is unchanged, so every test keeps its path and its identity.

use crate::*;

/// The constructor reports rather than aborts. Both profiles are asserted,
/// because the whole point of the class is that they differ and only one of
/// them was ever looked at.
#[test]
fn debug_new_reports_a_degenerate_edge_instead_of_panicking() {
    let v = Vertex::new(());
    let outcome = Edge::debug_new(&v, &v, ());
    if cfg!(debug_assertions) {
        assert_eq!(
            outcome.err(),
            Some(errors::Error::SameVertex),
            "a degenerate edge must come back as a typed refusal, not a panic",
        );
    } else {
        assert!(
            outcome.is_ok(),
            "release still skips the check -- the C9 face of the class",
        );
    }
}

/// A well-formed edge is unmoved in either profile.
#[test]
fn debug_new_accepts_a_well_formed_edge() {
    let v = Vertex::from_points([(); 2]);
    let edge = Edge::debug_new(&v[0], &v[1], ()).expect("distinct vertices");
    assert_eq!(edge.front(), &v[0]);
    assert_eq!(edge.back(), &v[1]);
}

/// `Edge::mapped` calls `new_unchecked` directly, and this pins WHY that is
/// sound rather than convenient: `Vertex::mapped` allocates a fresh `Arc`
/// per call and `Vertex` equality is `Arc` pointer identity, so the mapped
/// ends of any edge -- even one whose points collapse onto each other --
/// are distinct vertices. The check `debug_new` would run is vacuous here.
#[test]
fn mapped_ends_are_distinct_vertices_even_when_the_points_collapse() {
    let v = Vertex::from_points([0_i32, 1]);
    let edge = Edge::new(&v[0], &v[1], ());
    let collapsed = edge.mapped(|_| 0_i32, |()| ());
    assert_eq!(collapsed.front().point(), collapsed.back().point());
    assert_ne!(
        collapsed.front(),
        collapsed.back(),
        "identity is the `Arc`, not the point -- so `front == back` is unreachable",
    );
}

// `Edge::concat`'s disposition is pinned by the TYPE rather than by a row
// here: it maps `debug_new`'s refusal onto its own `ConcatError::SameVertex`
// (`map_err`), so there is no `unwrap` for a test to catch failing, and the
// case is already rejected by the guard above the call. Exercising it needs
// a curve type implementing `Concat + Invertible + ParameterTransform`,
// which this crate has none of -- `monstertruck-modeling`'s suite covers
// `concat` on real curves.
