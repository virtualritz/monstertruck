//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

use super::*;

fn direction(ratios: [f64; 3]) -> Direction {
    Direction {
        label: String::new(),
        direction_ratios: ratios.to_vec(),
    }
}

/// Regression for `axis ∥ ref_direction`: ISO 10303 allows the placement's
/// `axis` and `ref_direction` to be parallel, but the Gram-Schmidt step
/// then normalized a zero vector and produced `NaN`, which later panicked
/// during meshing (`tolerance must be no less than 1e-6`). The conversion
/// must yield a finite, orthonormal basis instead.
#[test]
fn axis2_placement3d_parallel_axis_and_ref_direction_is_finite() {
    let placement = Axis2Placement3d {
        label: String::new(),
        location: CartesianPoint {
            label: String::new(),
            coordinates: vec![1.0, 2.0, 3.0],
        },
        axis: Some(direction([0.0, 0.0, 1.0])),
        // Parallel to `axis` -- the degenerate case.
        ref_direction: Some(direction([0.0, 0.0, 1.0])),
    };

    let matrix = Matrix4::from(&placement);
    let (x, y, z) = (
        matrix.x.truncate(),
        matrix.y.truncate(),
        matrix.z.truncate(),
    );
    for component in [x.x, x.y, x.z, y.x, y.y, y.z, z.x, z.y, z.z] {
        assert!(component.is_finite(), "placement basis must be finite");
    }
    // The recovered basis is orthonormal.
    assert!((x.magnitude() - 1.0).abs() < 1.0e-9);
    assert!((y.magnitude() - 1.0).abs() < 1.0e-9);
    assert!(x.dot(z).abs() < 1.0e-9);
    assert!(y.dot(z).abs() < 1.0e-9);
}
