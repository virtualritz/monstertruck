//! **Deprecated.** STEP support moved into [`monstertruck_io`], behind its
//! `step` feature.
//!
//! Every format monstertruck reads or writes now lives in one crate with a
//! feature per format, so a caller reaches STEP, IGES and whatever follows
//! through a single dependency instead of one crate per format. This crate is a
//! re-export kept so that an existing `monstertruck-step = "0.3"` requirement
//! keeps resolving and compiling.
//!
//! Migration is a dependency swap and a name change; the API is unchanged:
//!
//! ```toml
//! # before
//! monstertruck-step = "0.3"
//! # after
//! monstertruck-io = { version = "0.3", features = ["step"] }
//! ```
//!
//! ```ignore
//! // before
//! use monstertruck_step::load::Table;
//! // after
//! use monstertruck_io::step::load::Table;
//! ```

#![deny(clippy::all, rust_2018_idioms)]

#[doc(inline)]
pub use monstertruck_io::step::*;
