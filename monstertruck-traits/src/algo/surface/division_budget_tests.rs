//! Unit tests for the parent module (`division_budget_tests`).
//!
//! Split out of the module file so the source stays readable. The module
//! name is unchanged, so every test keeps its path and its identity.

use super::*;

/// A surface whose evaluation carries a high-frequency term that a bilinear
/// interpolant can never match, so the deviation test fails at EVERY level:
/// the stand-in for the real non-converging case (a rational B-spline asked
/// to flatten a parameter box outside its knot vector, spec 012 U1.0).
#[derive(Clone, Copy, Debug)]
struct NonConvergingSurface;

impl ParametricSurface for NonConvergingSurface {
    type Point = Point3;
    type Vector = Vector3;
    fn evaluate(&self, u: f64, v: f64) -> Point3 {
        // 1e9 cycles across the unit domain: no level of halving resolves it.
        Point3::new(u, v, f64::sin(1.0e9 * u) * f64::cos(1.0e9 * v))
    }
    fn derivative_u(&self, _: f64, _: f64) -> Vector3 { Vector3::unit_x() }
    fn derivative_v(&self, _: f64, _: f64) -> Vector3 { Vector3::unit_y() }
    fn derivative_uu(&self, _: f64, _: f64) -> Vector3 { Vector3::zero() }
    fn derivative_uv(&self, _: f64, _: f64) -> Vector3 { Vector3::zero() }
    fn derivative_vv(&self, _: f64, _: f64) -> Vector3 { Vector3::zero() }
    fn derivative_mn(&self, m: usize, n: usize, u: f64, v: f64) -> Vector3 {
        match (m, n) {
            (0, 0) => self.evaluate(u, v).to_vec(),
            (1, 0) => self.derivative_u(u, v),
            (0, 1) => self.derivative_v(u, v),
            _ => Vector3::zero(),
        }
    }
}

/// A plane: the control. Bilinear interpolation is EXACT on it, so the very
/// first deviation test passes and the division is the two endpoints.
#[derive(Clone, Copy, Debug)]
struct FlatSurface;

impl ParametricSurface for FlatSurface {
    type Point = Point3;
    type Vector = Vector3;
    fn evaluate(&self, u: f64, v: f64) -> Point3 { Point3::new(u, v, 0.0) }
    fn derivative_u(&self, _: f64, _: f64) -> Vector3 { Vector3::unit_x() }
    fn derivative_v(&self, _: f64, _: f64) -> Vector3 { Vector3::unit_y() }
    fn derivative_uu(&self, _: f64, _: f64) -> Vector3 { Vector3::zero() }
    fn derivative_uv(&self, _: f64, _: f64) -> Vector3 { Vector3::zero() }
    fn derivative_vv(&self, _: f64, _: f64) -> Vector3 { Vector3::zero() }
    fn derivative_mn(&self, m: usize, n: usize, u: f64, v: f64) -> Vector3 {
        match (m, n) {
            (0, 0) => self.evaluate(u, v).to_vec(),
            (1, 0) => self.derivative_u(u, v),
            (0, 1) => self.derivative_v(u, v),
            _ => Vector3::zero(),
        }
    }
}

#[test]
fn a_non_converging_surface_terminates_instead_of_grinding() {
    // Before the cell budget this ran to `MAX_PARAMETER_DIVISION_RECURSION`
    // = 100 levels of a TENSOR-PRODUCT grid -- `4^100` implied cells, which
    // is not a bound (ledger M10). The depth cap is still there; this asserts
    // the one that binds.
    let budgeted =
        parameter_division_with_budget(&NonConvergingSurface, ((0.0, 1.0), (0.0, 1.0)), 0.01, 4096);
    assert!(
        budgeted.truncated,
        "a surface that cannot converge must trip the budget"
    );
    assert!(
        budgeted.cells <= 4096,
        "spend {} must not exceed the budget",
        budgeted.cells,
    );
    // The returned grid is still a real, level-complete division of the range.
    let (udiv, vdiv) = &budgeted.division;
    assert_eq!(udiv.first(), Some(&0.0));
    assert_eq!(udiv.last(), Some(&1.0));
    assert_eq!(vdiv.first(), Some(&0.0));
    assert_eq!(vdiv.last(), Some(&1.0));
    assert!(udiv.windows(2).all(|window| window[0] < window[1]));
    assert!(vdiv.windows(2).all(|window| window[0] < window[1]));
}

#[test]
fn the_infallible_entry_returns_a_coarse_division_and_the_fallible_one_refuses() {
    // The contract for a caller with no refusal channel (a viewer, a volume
    // estimate) versus one whose SOUNDNESS rests on the chord bound.
    let coarse = parameter_division(&NonConvergingSurface, ((0.0, 1.0), (0.0, 1.0)), 0.01);
    assert!(coarse.0.len() >= 2 && coarse.1.len() >= 2);
    let refused = try_parameter_division(&NonConvergingSurface, ((0.0, 1.0), (0.0, 1.0)), 0.01);
    let error = refused.expect_err("a truncated division must not be reported as Ok");
    assert_eq!(error.budget, MAX_PARAMETER_DIVISION_CELLS);
    assert!(error.cells <= MAX_PARAMETER_DIVISION_CELLS);
    assert!(
        error.to_string().contains("cell budget"),
        "the refusal must name the stage: {error}",
    );
    // Same grid either way -- the fallible entry adds a verdict, not a
    // different division.
    assert_eq!(error.division, coarse);
}

#[test]
fn a_converging_surface_is_untouched_by_the_budget() {
    // The headroom assertion in miniature: a division that terminates must
    // spend the same cells and return the same grid at ANY budget above its
    // spend, including the production one.
    let range = ((0.0, 1.0), (0.0, 1.0));
    let tiny = parameter_division_with_budget(&FlatSurface, range, 0.01, 8);
    let production =
        parameter_division_with_budget(&FlatSurface, range, 0.01, MAX_PARAMETER_DIVISION_CELLS);
    assert!(!tiny.truncated);
    assert!(!production.truncated);
    assert_eq!(tiny.cells, production.cells);
    assert_eq!(tiny.division, production.division);
    assert_eq!(tiny.division, (vec![0.0, 1.0], vec![0.0, 1.0]));
    assert_eq!(tiny.cells, 1, "one level, one cell, no refinement");
}

#[test]
fn the_work_meter_charges_what_the_division_spends() {
    let _ = take_division_work();
    let _ = take_division_totals();
    let budgeted =
        parameter_division_with_budget(&NonConvergingSurface, ((0.0, 1.0), (0.0, 1.0)), 0.01, 1024);
    let work = take_division_work();
    assert_eq!(work.cells, budgeted.cells);
    assert!(work.truncated);
    let (total, truncations) = take_division_totals();
    assert_eq!(total, budgeted.cells);
    assert_eq!(truncations, 1);
}
