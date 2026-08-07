//! Unit tests for the parent module (`basis_window_tests`).
//!
//! Split out of the module file so the source stays readable. The module
//! name is unchanged, so every test keeps its path and its identity.

use super::BasisWindow;
use smallvec::smallvec;

#[test]
fn empty_window_round_trips_to_zero_vector() {
    let window = BasisWindow::empty(5);
    assert!(window.is_empty());
    assert_eq!(window.len(), 0);
    assert_eq!(window.start_index(), 0);
    assert_eq!(window.total_length(), 5);
    assert_eq!(window.values(), &[]);
    assert_eq!(window.to_full_values(), vec![0.0; 5]);
}

#[test]
fn dense_window_reconstructs_full_vector_at_start_offset() {
    let window = BasisWindow::new(2, smallvec![0.5, 0.3, 0.2], 6);
    assert_eq!(window.start_index(), 2);
    assert_eq!(window.len(), 3);
    assert_eq!(window.values(), &[0.5, 0.3, 0.2]);
    assert_eq!(window.total_length(), 6);
    assert_eq!(window.to_full_values(), vec![0.0, 0.0, 0.5, 0.3, 0.2, 0.0]);
}

#[test]
fn as_ref_yields_inner_values() {
    let window = BasisWindow::new(1, smallvec![1.0, 0.0, 0.5], 4);
    let as_ref: &[f64] = window.as_ref();
    assert_eq!(as_ref, &[1.0, 0.0, 0.5]);
}

#[test]
fn window_at_end_pads_zeros_after_last_value() {
    let window = BasisWindow::new(3, smallvec![0.25, 0.5], 5);
    assert_eq!(window.to_full_values(), vec![0.0, 0.0, 0.0, 0.25, 0.5]);
}
