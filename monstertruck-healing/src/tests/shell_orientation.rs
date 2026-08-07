use super::*;
use monstertruck_topology::Shell;

// ---------------------------------------------------------------------------
// Shell orientation normalization (Campaign 7A.1)
// ---------------------------------------------------------------------------

/// Minimal closed manifold shell: a tetrahedron with consistently oriented
/// faces (every edge traversed once in each direction). Unit geometry -- the
/// normalizer is purely topological.
fn oriented_tetrahedron() -> Shell<(), (), ()> {
    use monstertruck_topology::{Edge, Face, Vertex, Wire};
    let v: Vec<Vertex<()>> = (0..4).map(|_| Vertex::new(())).collect();
    let edge = |a: usize, b: usize| Edge::new(&v[a], &v[b], ());
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
        Face::new(vec![wire([e03.clone(), e13.inverse(), e01.inverse()])], ()),
        Face::new(vec![wire([e13.clone(), e23.inverse(), e12.inverse()])], ()),
    ]
    .into_iter()
    .collect()
}

#[test]
fn normalize_shell_orientation_repairs_flipped_face() {
    use monstertruck_topology::shell::ShellCondition;
    let mut shell = oriented_tetrahedron();
    assert_eq!(shell.shell_condition(), ShellCondition::Closed);

    shell[2].invert();
    assert_eq!(
        shell.shell_condition(),
        ShellCondition::Regular,
        "a single flipped face must demote the shell to Regular",
    );

    let outcome = normalize_shell_orientation(&mut shell);
    assert_eq!(outcome.flipped_faces, 1);
    assert_eq!(outcome.conflicts, 0);
    assert_eq!(outcome.irregular_edges, 0);
    assert_eq!(shell.shell_condition(), ShellCondition::Closed);
}

#[test]
fn normalize_shell_orientation_keeps_consistent_shell_untouched() {
    use monstertruck_topology::shell::ShellCondition;
    let mut shell = oriented_tetrahedron();
    let orientations: Vec<bool> = shell.iter().map(|face| face.orientation()).collect();

    let outcome = normalize_shell_orientation(&mut shell);
    assert_eq!(outcome, OrientationNormalization::default());
    assert_eq!(shell.shell_condition(), ShellCondition::Closed);
    let after: Vec<bool> = shell.iter().map(|face| face.orientation()).collect();
    assert_eq!(orientations, after, "no face may be touched");
}

#[test]
fn normalize_shell_orientation_majority_flip_converges() {
    use monstertruck_topology::shell::ShellCondition;
    let mut shell = oriented_tetrahedron();
    // Flip three of four faces: the flood fill keeps the FIRST face's side
    // (face 0, still original here), so the three flipped faces flip back
    // (global outwardness is out of scope -- an all-flipped shell would be
    // equally Closed).
    shell[1].invert();
    shell[2].invert();
    shell[3].invert();
    assert_eq!(shell.shell_condition(), ShellCondition::Regular);

    let outcome = normalize_shell_orientation(&mut shell);
    assert_eq!(outcome.conflicts, 0);
    assert_eq!(outcome.flipped_faces, 3);
    assert_eq!(shell.shell_condition(), ShellCondition::Closed);
}
