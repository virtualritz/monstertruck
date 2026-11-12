//! Import failures, typed.

/// What can go wrong importing an exchange file.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum Error {
    /// The file could not be read.
    #[error("reading the input failed")]
    Io(#[from] std::io::Error),

    /// The decoder rejected the file, or recovered nothing this crate can use.
    ///
    /// Carries the decoder's own message: those messages name the offending
    /// entity, and discarding them would make a defect much harder to find.
    #[error("{format} decode failed: {message}")]
    Decode {
        /// Which format was being read.
        format: &'static str,
        /// The decoder's own diagnostic.
        message: String,
    },

    /// The file decoded, but carried nothing this crate can turn into a solid.
    ///
    /// Distinct from [`Error::Decode`]: the input was understood and simply held
    /// no usable body, which is a fact about the file rather than a failure.
    #[error("{format} decoded but carried no convertible geometry")]
    NoGeometry {
        /// Which format was being read.
        format: &'static str,
    },

    /// The file carried a body kind this crate cannot represent.
    ///
    /// Measured 2026-08-06 against cadmpeg 0.4: STEP yields `solid` bodies, and
    /// IGES yields `sheet` and `wire`. A sheet becomes a
    /// [`ImportedBody::Sheet`]; a wire body is a curve collection with no faces
    /// and monstertruck's compressed types have nowhere to put it, so it is
    /// refused BY NAME rather than dropped from the returned list. Silently
    /// returning fewer bodies than the file holds is the failure this variant
    /// exists to prevent.
    ///
    /// [`ImportedBody::Sheet`]: crate::cadmpeg::ImportedBody::Sheet
    #[error("{format} carried a {kind} body, which has no monstertruck equivalent")]
    UnsupportedBodyKind {
        /// Which format was being read.
        format: &'static str,
        /// The body kind cadmpeg reported.
        kind: &'static str,
    },

    /// The file carried a surface kind with no exact monstertruck equivalent.
    ///
    /// Refused rather than approximated. A cone arriving as a spline patch, or a
    /// source-native triangle soup arriving as a plane, is a loss of exactness
    /// that the boolean kernel pays for much later and far from here. The caller
    /// is told which kind was refused so the file can be re-exported in a form
    /// this crate reads.
    #[error("{format} carried a {kind} surface, which has no exact monstertruck equivalent")]
    UnsupportedSurfaceKind {
        /// Which format was being read.
        format: &'static str,
        /// The surface kind cadmpeg reported.
        kind: &'static str,
    },

    /// The file carried a curve kind with no exact monstertruck equivalent.
    ///
    /// Same reasoning as [`Error::UnsupportedSurfaceKind`].
    #[error("{format} carried a {kind} curve, which has no exact monstertruck equivalent")]
    UnsupportedCurveKind {
        /// Which format was being read.
        format: &'static str,
        /// The curve kind cadmpeg reported.
        kind: &'static str,
    },

    /// A placement in the product structure cannot be applied.
    ///
    /// Distinct from [`Error::UnsupportedBodyKind`]: the body kind is fine and
    /// it is the OCCURRENCE's placement that is refused, either because a
    /// per-axis scale has no defined composition order or because the composed
    /// matrix is singular. A caller matching on body kind must not be told an
    /// occurrence problem is a body-kind problem.
    #[error("{format} carried an occurrence placement that cannot be applied: {kind}")]
    UnplaceableOccurrence {
        /// Which format was being read.
        format: &'static str,
        /// The stable refusal word.
        kind: &'static str,
    },

    /// A reference in the decoded graph named an entity the document lacks.
    ///
    /// The intermediate representation is a table of entities that refer to each
    /// other by string id. A reference with no target means the decoded document
    /// is not closed, so converting the part of it that IS closed would hand back
    /// a body with holes in it.
    #[error("{format} referenced a {kind} that the decoded document does not contain: {id}")]
    DanglingReference {
        /// Which format was being read.
        format: &'static str,
        /// Which entity table the missing target belongs to.
        kind: &'static str,
        /// The id that resolved to nothing.
        id: String,
    },

    /// Geometry decoded, but its own numbers do not describe a usable shape.
    ///
    /// A zero-length axis, a knot vector too short for its control net, a radius
    /// that is not finite. Separate from [`Error::Decode`]: the bytes parsed and
    /// the entity is the kind it claims to be, but the values cannot be built
    /// into geometry.
    #[error("{format} carried malformed geometry: {detail}")]
    MalformedGeometry {
        /// Which format was being read.
        format: &'static str,
        /// What is wrong, naming the entity where the converter can.
        detail: String,
    },

    /// This path is not written yet.
    ///
    /// Present so a placeholder cannot be mistaken for a successful import that
    /// happened to find nothing. It is deliberately unpleasant to ignore.
    #[error("{what} is not implemented yet")]
    Unimplemented {
        /// The conversion that is missing.
        what: &'static str,
    },
}

/// Import result.
pub type Result<T> = std::result::Result<T, Error>;
