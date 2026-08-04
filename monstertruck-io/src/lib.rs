//! Import CAD exchange formats into monstertruck's B-rep types.
//!
//! # Status
//!
//! The `step` feature is complete: it re-exports monstertruck's own reader and
//! writer, which work today.
//!
//! The `iges` feature is a **placeholder**. It decodes a real file with a real
//! decoder, but the last step -- turning the recovered geometry into
//! monstertruck's B-rep -- is unwritten, so it returns
//! [`Error::Unimplemented`] rather than an empty model. A caller therefore
//! cannot mistake "not written yet" for "this file contained no geometry", and
//! the two are reported by distinct variants. That distinction is the point: an
//! importer that silently returns nothing is worse than one that refuses, and
//! `tests/refuses_rather_than_returns_nothing.rs` fails the day it stops.
//!
//! # Why one crate and not one per format
//!
//! The [`cadmpeg`](https://github.com/cadmpeg/cadmpeg) codecs -- IGES today,
//! others later -- all decode into a single intermediate representation,
//! `cadmpeg_ir::CadIr`. The per-format work is therefore only *which decoder to
//! call*; the part that carries the risk, mapping recovered geometry and
//! topology onto [`monstertruck_topology`], is shared. So this crate holds one
//! converter in [`cadmpeg`] and a thin module per format on top of it.
//!
//! Each format is a feature, and none is on by default: a consumer that wants
//! STEP should not compile an IGES reader.
//!
//! # Division of labour
//!
//! | format | feature | decoder | why |
//! | --- | --- | --- | --- |
//! | ISO 10303-21 (STEP) | `step` | [`monstertruck_step`] | monstertruck's own reader and writer |
//! | IGES 5.3 | `iges` | `cadmpeg-codec-iges` | nothing to gain from writing our own |
//!
//! STEP deliberately does **not** go through cadmpeg. monstertruck's own STEP
//! reader is measured against a corpus, routes analytic surfaces onto closed
//! forms so booleans stay exact, and reports what a file lost. Measured
//! 2026-08-04 on `occt-cylinder.step`: monstertruck reads it correctly, and
//! cadmpeg drops the whole `MANIFOLD_SOLID_BREP` because one parameter-space
//! `CIRCLE` fails to decode (cadmpeg/cadmpeg#79). Writing STEP stays ours
//! outright.
//!
//! IGES is the opposite case: monstertruck cannot read it at all, and cadmpeg
//! scores it highly, so "imperfect but loud" beats "absent".

#![cfg_attr(not(debug_assertions), deny(warnings))]
#![deny(clippy::all, rust_2018_idioms)]
#![warn(
    missing_docs,
    missing_debug_implementations,
    trivial_casts,
    trivial_numeric_casts,
    unsafe_code,
    unused_import_braces,
    unused_qualifications
)]

mod error;

pub use error::{Error, Result};

#[cfg(feature = "iges")]
pub mod cadmpeg;
#[cfg(feature = "iges")]
pub mod iges;
#[cfg(feature = "step")]
pub mod step;
