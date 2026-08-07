//! Deterministic content hashing for the modeling geometry enums.

use super::*;

pub(super) fn content_hash64<T: DeterministicContentHash>(value: &T) -> u64 {
    let mut hasher = ContentHasher::default();
    value.content_hash(&mut hasher);
    hasher.finish()
}

// ---------------------------------------------------------------------------
// Deterministic content hashing for modeling enums.
// ---------------------------------------------------------------------------

impl DeterministicContentHash for Conic2D {
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        match self {
            Self::Ellipse(v) => {
                state.write_u8(0);
                v.content_hash(state);
            }
            Self::Hyperbola(v) => {
                state.write_u8(1);
                v.content_hash(state);
            }
            Self::Parabola(v) => {
                state.write_u8(2);
                v.content_hash(state);
            }
        }
    }
}

impl DeterministicContentHash for Curve2D {
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        match self {
            Self::Line(v) => {
                state.write_u8(0);
                v.content_hash(state);
            }
            Self::Polyline(v) => {
                state.write_u8(1);
                v.content_hash(state);
            }
            Self::Conic(v) => {
                state.write_u8(2);
                v.content_hash(state);
            }
            Self::BsplineCurve(v) => {
                state.write_u8(3);
                v.content_hash(state);
            }
            Self::NurbsCurve(v) => {
                state.write_u8(4);
                v.content_hash(state);
            }
        }
    }
}

impl DeterministicContentHash for Curve {
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        match self {
            Self::Line(v) => {
                state.write_u8(0);
                v.content_hash(state);
            }
            Self::BsplineCurve(v) => {
                state.write_u8(1);
                v.content_hash(state);
            }
            Self::NurbsCurve(v) => {
                state.write_u8(2);
                v.content_hash(state);
            }
            Self::ParameterCurve(v) => {
                state.write_u8(3);
                v.content_hash(state);
            }
            Self::IntersectionCurve(v) => {
                state.write_u8(4);
                v.content_hash(state);
            }
        }
    }
}

impl DeterministicContentHash for Surface {
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        match self {
            Self::Plane(v) => {
                state.write_u8(0);
                v.content_hash(state);
            }
            Self::BsplineSurface(v) => {
                state.write_u8(1);
                v.content_hash(state);
            }
            Self::NurbsSurface(v) => {
                state.write_u8(2);
                v.content_hash(state);
            }
            Self::RevolutionSurface(v) => {
                state.write_u8(3);
                v.content_hash(state);
            }
            Self::TsplineSurface(v) => {
                state.write_u8(4);
                v.content_hash(state);
            }
            // New tags only. 0..=4 keep their meaning, so every solid that
            // carries no sphere and no torus hashes byte-identically.
            Self::SphericalSurface(v) => {
                state.write_u8(5);
                v.content_hash(state);
            }
            Self::ToroidalSurface(v) => {
                state.write_u8(6);
                v.content_hash(state);
            }
        }
    }
}
