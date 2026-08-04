//! Assembly data structures using a directed acyclic graph (DAG).
//!
//! # Examples
//!
//! ```
//! use monstertruck_assembly::assy::*;
//!
//! let mut assy = Assembly::<(), (), f64, ()>::new();
//!
//! // Create nodes and connect with transform edges
//! let nodes = assy.create_nodes([().into(); 4]);
//! assy.create_edge(nodes[0], nodes[1], 2.0.into());
//! assy.create_edge(nodes[1], nodes[2], 3.0.into());
//! assy.create_edge(nodes[2], nodes[3], 5.0.into());
//!
//! // Walk the path and accumulate transforms
//! let path = assy.maximal_paths_iter(nodes[0]).next().unwrap();
//! assert_eq!(path.matrix(), 30.0); // 2 * 3 * 5
//! ```

#![deny(clippy::all, rust_2018_idioms)]
#![warn(
    missing_docs,
    missing_debug_implementations,
    trivial_casts,
    trivial_numeric_casts,
    unsafe_code,
    unstable_features,
    unused_import_braces,
    unused_qualifications
)]

/// Assembly structure
pub mod assy;
/// Abstract DAG structure
pub mod dag;
