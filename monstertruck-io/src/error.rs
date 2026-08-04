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
