//! Placements: turning the intermediate representation's axis-and-reference
//! triples into the matrices monstertruck's analytic geometry is posed by.
//!
//! Every analytic carrier in `cadmpeg-ir` is described the STEP way -- an origin,
//! an axis, and a zero-azimuth reference direction. monstertruck poses the same
//! shapes with a [`Matrix4`] on a [`Processor`], or with three points on a
//! [`Plane`]. This module is the only place that bridges the two, so the
//! handedness argument below is made once.
//!
//! [`Processor`]: monstertruck_geometry::prelude::Processor
//! [`Plane`]: monstertruck_modeling::Plane

use cadmpeg_ir::features::FinitePoint3;
use cadmpeg_ir::math::{Point3 as IrPoint3, Vector3 as IrVector3};
use cadmpeg_ir::transform::Transform as IrTransform;
use cadmpeg_ir::units::OrthonormalFrame3;
use monstertruck_modeling::{InnerSpace, Matrix4, Point3, Vector3};

/// Coordinates as monstertruck spells them.
///
/// cadmpeg normalises lengths to millimetres on decode, so there is no unit
/// factor to apply here -- see the module note on [`crate::cadmpeg`].
pub(super) fn point(value: &IrPoint3) -> Point3 { Point3::new(value.x, value.y, value.z) }

/// An admitted (finite) intermediate-representation point.
pub(super) fn finite_point(value: FinitePoint3) -> Point3 { point(&value.get()) }

/// A direction as monstertruck spells it, not yet normalised.
pub(super) fn vector(value: &IrVector3) -> Vector3 { Vector3::new(value.x, value.y, value.z) }

/// A row-major intermediate-representation transform as a column-major
/// [`Matrix4`].
///
/// `Transform::rows()[i][j]` is row `i`, column `j`; [`Matrix4::from_cols`]
/// wants columns. Getting this backwards transposes every placement, which for a
/// rigid motion inverts the rotation and leaves the translation in the wrong row
/// -- so it is not the kind of mistake that shows up as a small error.
///
/// Since cadmpeg 0.6 a `Transform` is affine by construction (its bottom row is
/// a fact of the type, not stored), so there is no projective case to refuse.
pub(super) fn matrix(value: &IrTransform) -> Matrix4 {
    let rows = value.rows();
    let column = |index: usize| {
        monstertruck_modeling::Vector4::new(
            rows[0][index],
            rows[1][index],
            rows[2][index],
            rows[3][index],
        )
    };
    Matrix4::from_cols(column(0), column(1), column(2), column(3))
}

/// A right-handed orthonormal placement: where a carrier sits, which way its
/// axis points, and where its parameter zero is.
#[derive(Debug, Clone, Copy)]
pub(super) struct Frame {
    /// The carrier's origin or centre.
    pub(super) origin: Point3,
    /// Unit axis.
    pub(super) axis: Vector3,
    /// Unit zero-azimuth direction, perpendicular to [`Frame::axis`].
    pub(super) reference: Vector3,
}

impl Frame {
    /// The third axis, completing a right-handed triple.
    pub(super) fn co_reference(&self) -> Vector3 { self.axis.cross(self.reference) }

    /// The matrix that maps the canonical frame -- x to `reference`, y to
    /// `axis x reference`, z to `axis`, origin to `origin` -- into place, with
    /// `scale` applied to the two radial columns.
    ///
    /// Scaling the radial columns is what turns a UNIT circle into a circle of a
    /// given radius without touching the entity, which is how monstertruck's
    /// exact conics are built: the radius lives in the placement, and the entity
    /// stays canonical and exactly representable.
    pub(super) fn placement(&self, scale: f64) -> Matrix4 {
        let radial = self.reference * scale;
        Matrix4::from_cols(
            radial.extend(0.0),
            (self.axis.cross(radial)).extend(0.0),
            self.axis.extend(0.0),
            self.origin.to_homogeneous(),
        )
    }
}

/// The placement of an analytic carrier.
///
/// Since cadmpeg 0.6 every analytic carrier holds an [`OrthonormalFrame3`],
/// which the type admits only when the axis and the reference are unit length
/// and perpendicular to `1e-9`. The zero-length and parallel-reference cases
/// this module used to refuse are therefore unrepresentable, and building a
/// frame cannot fail.
///
/// The reference is still projected perpendicular to the axis and both are
/// renormalised. That costs nothing and removes the `1e-9` admission slack, so
/// the placement matrix below is orthonormal to rounding rather than to the
/// source's tolerance.
pub(super) fn frame(origin: FinitePoint3, frame: &OrthonormalFrame3) -> Frame {
    let axis = vector(frame.axis().as_raw()).normalize();
    let reference = vector(frame.reference().as_raw());
    Frame {
        origin: finite_point(origin),
        axis,
        reference: (reference - axis * axis.dot(reference)).normalize(),
    }
}
