//! [`DeterministicContentHash`] implementations for geometry types.

use std::hash::Hasher;

use monstertruck_core::DeterministicContentHash;
use monstertruck_core::One;

use crate::decorators::*;
use crate::nurbs::*;
use crate::specifieds::*;

// ---------------------------------------------------------------------------
// NURBS primitives.
// ---------------------------------------------------------------------------

impl DeterministicContentHash for KnotVector {
    fn content_hash<H: Hasher>(&self, state: &mut H) { self.as_slice().content_hash(state); }
}

impl<P: DeterministicContentHash> DeterministicContentHash for BsplineCurve<P> {
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        self.knot_vector().content_hash(state);
        self.control_points().content_hash(state);
    }
}

impl<V: DeterministicContentHash> DeterministicContentHash for NurbsCurve<V> {
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        self.knot_vector().content_hash(state);
        self.control_points().content_hash(state);
    }
}

impl<P: DeterministicContentHash> DeterministicContentHash for BsplineSurface<P> {
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        self.knot_vectors().content_hash(state);
        self.control_points().content_hash(state);
    }
}

impl<V: DeterministicContentHash> DeterministicContentHash for NurbsSurface<V> {
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        self.knot_vectors().content_hash(state);
        self.control_points().content_hash(state);
    }
}

// ---------------------------------------------------------------------------
// Specified primitives.
// ---------------------------------------------------------------------------

impl<P: DeterministicContentHash> DeterministicContentHash for Line<P> {
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        self.0.content_hash(state);
        self.1.content_hash(state);
    }
}

impl DeterministicContentHash for Plane {
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        self.origin().content_hash(state);
        // Hash u_axis and v_axis -- derived from the stored p,q fields.
        self.axis_u().content_hash(state);
        self.axis_v().content_hash(state);
    }
}

impl DeterministicContentHash for Sphere {
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        self.center().content_hash(state);
        self.radius().content_hash(state);
    }
}

impl DeterministicContentHash for Torus {
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        self.center().content_hash(state);
        self.large_radius().content_hash(state);
        self.small_radius().content_hash(state);
    }
}

impl<P> DeterministicContentHash for UnitCircle<P> {
    fn content_hash<H: Hasher>(&self, _state: &mut H) {
        // Phantom-data type -- no semantic content.
    }
}

impl<P> DeterministicContentHash for UnitHyperbola<P> {
    fn content_hash<H: Hasher>(&self, _state: &mut H) {
        // Phantom-data type -- no semantic content.
    }
}

impl<P> DeterministicContentHash for UnitParabola<P> {
    fn content_hash<H: Hasher>(&self, _state: &mut H) {
        // Phantom-data type -- no semantic content.
    }
}

// ---------------------------------------------------------------------------
// Decorators.
// ---------------------------------------------------------------------------

impl<E: DeterministicContentHash, T: DeterministicContentHash + One> DeterministicContentHash
    for Processor<E, T>
{
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        self.entity().content_hash(state);
        self.transform().content_hash(state);
        self.orientation().content_hash(state);
    }
}

impl<C: DeterministicContentHash> DeterministicContentHash for TrimmedCurve<C> {
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        self.curve().content_hash(state);
        self.range().content_hash(state);
    }
}

impl<C: DeterministicContentHash> DeterministicContentHash for RevolutionSurface<C> {
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        self.entity_curve().content_hash(state);
        self.origin().content_hash(state);
        self.axis().content_hash(state);
    }
}

impl<C: DeterministicContentHash, V: Copy + DeterministicContentHash> DeterministicContentHash
    for ExtrusionSurface<C, V>
{
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        self.entity_curve().content_hash(state);
        self.extruding_vector().content_hash(state);
    }
}

impl<C: DeterministicContentHash, S: DeterministicContentHash> DeterministicContentHash
    for ParameterCurve<C, S>
{
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        self.curve().content_hash(state);
        self.surface().content_hash(state);
    }
}

impl<C: DeterministicContentHash, S0: DeterministicContentHash, S1: DeterministicContentHash>
    DeterministicContentHash for IntersectionCurve<C, S0, S1>
{
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        self.leader().content_hash(state);
        self.surface0().content_hash(state);
        self.surface1().content_hash(state);
    }
}

impl<
    C: DeterministicContentHash,
    S0: DeterministicContentHash,
    S1: DeterministicContentHash,
    T0: DeterministicContentHash,
    T1: DeterministicContentHash,
> DeterministicContentHash for SurfaceCurve<C, S0, S1, T0, T1>
{
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        self.leader().content_hash(state);
        self.surface0().content_hash(state);
        self.surface1().content_hash(state);
        self.boundary0().content_hash(state);
        self.boundary1().content_hash(state);
    }
}

// ---------------------------------------------------------------------------
// T-spline.
// ---------------------------------------------------------------------------

impl<P: DeterministicContentHash> DeterministicContentHash for crate::t_spline::Tmesh<P> {
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        use crate::t_spline::TmeshDirection;

        let cps = self.control_points();
        state.write_usize(cps.len());

        let dirs = [
            TmeshDirection::Up,
            TmeshDirection::Right,
            TmeshDirection::Down,
            TmeshDirection::Left,
        ];

        cps.iter().enumerate().for_each(|(idx, cp)| {
            let guard = cp.read();
            guard.point().content_hash(state);
            guard.knot_coordinates().content_hash(state);

            // Hash connectivity: for each direction, encode connection type
            // and neighbor index (resolved via Arc identity against the
            // control-point vector).
            dirs.iter().for_each(|&dir| {
                match guard.get(dir) {
                    None => {
                        // T-junction.
                        state.write_u8(0);
                    }
                    Some((None, weight)) => {
                        // Edge condition.
                        state.write_u8(1);
                        weight.content_hash(state);
                    }
                    Some((Some(neighbor), weight)) => {
                        // Connected to another control point.
                        state.write_u8(2);
                        weight.content_hash(state);
                        // Resolve neighbor index in the control-point vector.
                        let neighbor_idx = cps
                            .iter()
                            .position(|other| std::sync::Arc::ptr_eq(neighbor, other))
                            .unwrap_or(idx);
                        neighbor_idx.content_hash(state);
                    }
                }
            });
        });
    }
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests;
