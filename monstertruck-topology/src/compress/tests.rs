//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

use super::*;

#[test]
fn cloned_without_trims_matches_clone_then_erase_trims() {
    let shell = CompressedTrimmedShell {
        vertices: vec![0usize, 1usize],
        edges: vec![CompressedEdge {
            vertices: (0, 1),
            curve: 5usize,
        }],
        faces: vec![CompressedTrimmedFace {
            boundaries: vec![vec![CompressedEdgeUse {
                index: 0,
                orientation: true,
                trim_curve: Some(7usize),
            }]],
            orientation: true,
            surface: (),
        }],
    };

    assert_eq!(shell.cloned_without_trims(), shell.clone().erase_trims());
}
