//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

use super::*;

/// Minimal closed manifold shell: a tetrahedron with consistently oriented
/// faces. Purely topological -- `()` for point, curve and surface.
fn tetrahedron() -> Shell<(), (), ()> {
    let vertex: Vec<Vertex<()>> = (0..4).map(|_| Vertex::new(())).collect();
    let edge = |a: usize, b: usize| Edge::new(&vertex[a], &vertex[b], ());
    let e01 = edge(0, 1);
    let e02 = edge(0, 2);
    let e03 = edge(0, 3);
    let e12 = edge(1, 2);
    let e13 = edge(1, 3);
    let e23 = edge(2, 3);
    let wire = |edges: [Edge<(), ()>; 3]| -> Wire<(), ()> { edges.to_vec().into() };
    [
        Face::new(vec![wire([e01.clone(), e12.clone(), e02.inverse()])], ()),
        Face::new(vec![wire([e02.clone(), e23.clone(), e03.inverse()])], ()),
        Face::new(vec![wire([e03, e13.inverse(), e01.inverse()])], ()),
        Face::new(vec![wire([e13, e23.inverse(), e12.inverse()])], ()),
    ]
    .into_iter()
    .collect()
}

fn trimmed(shell: Shell<(), (), ()>) -> TrimmedShell<(), (), (), ()> { TrimmedShell::from(shell) }

/// `TrimmedSolid::try_new` must accept exactly what `Solid::try_new`
/// accepts. Both are checked on the same shell so the two cannot drift
/// apart silently -- they share one definition
/// (`Shell::check_solid_boundary`) precisely so that this assertion holds.
#[test]
fn try_new_accepts_a_closed_manifold_boundary_just_as_solid_try_new_does() {
    let shell = tetrahedron();
    assert!(Solid::try_new(vec![shell.clone()]).is_ok());
    assert!(TrimmedSolid::try_new(vec![trimmed(shell)]).is_ok());
}

/// The C11 shape: a boundary shell one face short -- what a typed surface
/// refusal upstream leaves behind. `TrimmedSolid::try_new` must refuse it
/// with the SAME typed error `Solid::try_new` gives, and the unchecked
/// `TrimmedSolid::new` must still build it, because that is what `new`
/// has always done and callers depend on it.
#[test]
fn try_new_refuses_a_boundary_shell_that_lost_a_face() {
    let mut shell = tetrahedron();
    shell.pop().expect("the tetrahedron has four faces");
    assert_eq!(shell.len(), 3);

    assert_eq!(
        Solid::try_new(vec![shell.clone()]).err(),
        Some(Error::NotClosedShell),
    );
    assert_eq!(
        TrimmedSolid::try_new(vec![trimmed(shell.clone())]).err(),
        Some(Error::NotClosedShell),
        "the trimmed path must refuse what the plain path refuses",
    );
    assert_eq!(
        TrimmedSolid::new(vec![trimmed(shell)]).boundaries().len(),
        1,
        "`new` is the unchecked constructor and must stay unchecked",
    );
}

/// Two disjoint tetrahedra presented as ONE boundary shell: connected-ness
/// and closedness are different failures and must stay distinguishable, so
/// a caller can tell "a face went missing" from "this is two solids".
#[test]
fn try_new_distinguishes_a_disconnected_boundary_from_an_open_one() {
    let mut shell = tetrahedron();
    for face in tetrahedron() {
        shell.push(face);
    }
    assert_eq!(
        TrimmedSolid::try_new(vec![trimmed(shell)]).err(),
        Some(Error::NotConnected),
    );
    assert_eq!(
        TrimmedSolid::try_new(vec![TrimmedShell::<(), (), (), ()>::from(Shell::new())]).err(),
        Some(Error::EmptyShell),
    );
}
