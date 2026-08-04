//! Reading and writing ISO 10303-21, via [`monstertruck_step`].
//!
//! STEP deliberately does not go through cadmpeg. monstertruck's own reader is
//! measured against a corpus, routes analytic surfaces onto closed forms so the
//! booleans stay exact, and reports what a file lost; and monstertruck owns the
//! writer outright. This module exists so a caller can reach every format through
//! one crate, not to reimplement anything.

pub use monstertruck_step::{load, save};
