//! **M**ultifarious **O**mnificence, **N**omenclature **S**tandardized,
//! **T**erminology **E**nhanced & **R**efactored **Truck** -- a **Ru**st
//! **C**ad **K**ernel.
//!
//! A Rust CAD kernel -- B-rep modeling, NURBS curves and surfaces, meshing,
//! and GPU rendering. Forked from
//! [`ricosjp/truck`](https://github.com/ricosjp/truck) and renamed.
//!
//! This is a meta-crate: it re-exports the workspace's `monstertruck-*`
//! sub-crates behind feature flags so downstream code can pull in exactly
//! the surface area it needs.
//!
//! ```toml
//! [dependencies]
//! monstertruck = { version = "0.3", features = ["full"] }
//! ```
//!
//! # Architecture
//!
//! The kernel is split along the classic B-rep stack -- numerical primitives
//! at the bottom, topology and modeling in the middle, file I/O and rendering
//! at the top.
//!
//! ## Numerical core
//!
//! - [`core`] -- `f64`-only linear algebra, tolerance helpers, bounding
//!   boxes, and the [`derivatives`](crate::core::derivatives) family used to
//!   evaluate B-spline/NURBS bases.
//! - [`traits`] -- the kernel's trait family
//!   ([`ParametricCurve`](traits::ParametricCurve),
//!   [`ParametricSurface`](traits::ParametricSurface),
//!   [`SearchParameter`](traits::SearchParameter), etc.) together with the
//!   scalar-generic v2 successor under
//!   [`v2`](traits::v2).
//! - [`derive`](mod@derive) -- proc-macros (`#[derive(StepFormat)]`, `#[derive(StepLength)]`)
//!   for the STEP back-end.
//!
//! ## Geometry and topology
//!
//! - [`geometry`] -- concrete curves and surfaces: Bezier, B-spline, NURBS,
//!   conics, the [`Processor`](geometry::decorators::Processor) decorator,
//!   the new offset family
//!   ([`OffsetCurve`](geometry::decorators::OffsetCurve),
//!   [`OffsetSurface`](geometry::decorators::OffsetSurface),
//!   [`NormalOffsetField`](geometry::decorators::NormalOffsetField)),
//!   and analytic specifieds such as
//!   [`Sphere`](geometry::specifieds::Sphere).
//! - [`topology`] -- the B-rep graph: [`Vertex`](topology::Vertex),
//!   [`Edge`](topology::Edge), [`Wire`](topology::Wire),
//!   [`Face`](topology::Face), [`Shell`](topology::Shell),
//!   [`Solid`](topology::Solid), plus their `Compressed*` flattened forms
//!   used for serialisation and file I/O.
//!
//! ## Meshing
//!
//! - [`mesh`] -- `PolygonMesh`, `StructuredMesh`, and the OBJ/STL
//!   serialisers.
//! - [`meshing`] -- tessellation algorithms that turn B-rep
//!   [`Shell`](topology::Shell)/[`Solid`](topology::Solid) into
//!   triangle soups for rendering or 3D printing.
//!
//! ## Modeling and boolean ops
//!
//! - [`modeling`] -- the high-level builder
//!   ([`builder`](modeling::builder)) for vertices, edges, wires,
//!   sweeps ([`extrude`](modeling::builder::extrude),
//!   [`revolve`](modeling::builder::revolve)), tangent arcs, and primitive
//!   shapes ([`cuboid`](modeling::primitive::cuboid)
//!   etc.).
//! - [`solid`] -- shape operators
//!   ([`and`](solid::and), [`or`](solid::or)) for boolean combinations of
//!   solids. This is the boolean KERNEL only, so an external SSI backend
//!   exporting the same surface can stand in for it with no consumer code
//!   change.
//! - [`fillet`] -- rolling-ball edge fillets, a post-CSG pass.
//! - [`healing`] -- shape healing for solids imported from other CAD
//!   systems, also post-CSG.
//! - [`assembly`] -- DAG-structured assemblies (`Node`/`Edge`) that group
//!   parts and the rigid transforms between them.
//!
//! ## File I/O and rendering
//!
//! - [`step`] -- STEP (ISO 10303-21) reader and writer for parts and
//!   assemblies.
//! - [`gpu`] -- `wgpu` device wrapper used by the renderer.
//! - [`render`] -- a small forward renderer that draws meshes produced by
//!   [`meshing`].
//!
//! # Example: subtract a cylinder from a box, then export STEP
//!
//! Build a cuboid and a cylinder, subtract the cylinder via boolean `AND`
//! against its inverse, and emit the result as a STEP string.
//!
//! ```
//! # #[cfg(not(all(feature = "step", feature = "solid")))]
//! # fn main() {}
//! # #[cfg(all(feature = "step", feature = "solid"))]
//! # fn main() -> anyhow::Result<()> {
//! use monstertruck::modeling::*;
//! use monstertruck::solid;
//! use monstertruck::step::save::{CompleteStepDisplay, StepHeaderDescriptor, StepModel};
//!
//! // Unit cuboid centred on the origin.
//! let cube: Solid = primitive::cuboid(BoundingBox::from_iter([
//!     Point3::new(-0.5, -0.5, 0.0),
//!     Point3::new(0.5, 0.5, 1.0),
//! ]));
//!
//! // Cylinder of radius 0.3, axis along +Z, that pierces the cube top to bottom.
//! let seed = builder::vertex(Point3::new(0.3, 0.0, -0.1));
//! let rim = builder::revolve(
//!     &seed,
//!     Point3::new(0.0, 0.0, -0.1),
//!     Vector3::unit_z(),
//!     builder::SweepAngle::Closed,
//!     4,
//! );
//! let base = builder::try_attach_plane(&[rim])?;
//! let mut cylinder: Solid = builder::extrude(&base, Vector3::unit_z() * 1.2);
//!
//! // Invert the cylinder so `and` performs a subtraction.
//! cylinder.not();
//! let punched = solid::and(&cube, &cylinder, 0.05)?;
//!
//! // Flatten the topology and render as STEP.
//! let compressed = punched.compress();
//! let step = CompleteStepDisplay::new(
//!     StepModel::from(&compressed),
//!     StepHeaderDescriptor {
//!         organization_system: "monstertruck-doc".to_owned(),
//!         ..Default::default()
//!     },
//! )
//! .to_string();
//!
//! assert!(step.starts_with("ISO-10303-21;"));
//! assert!(step.contains("MANIFOLD_SOLID_BREP"));
//! # Ok(())
//! # }
//! ```
//!
//! # Feature flags
//!
//! Each sub-crate is gated behind a feature with the same name.
//!
//! | Feature    | Crate                   | Also enables                    |
//! | ---------- | ----------------------- | ------------------------------- |
//! | _(always)_ | `monstertruck-core`     |                                 |
//! | [`traits`]   | `monstertruck-traits`   |                                 |
//! | `derive`   | `monstertruck-derive`   |                                 |
//! | [`geometry`] | `monstertruck-geometry` | [`traits`]                        |
//! | [`topology`] | `monstertruck-topology` |                                 |
//! | [`mesh`]     | `monstertruck-mesh`     | [`traits`]                        |
//! | [`modeling`] | `monstertruck-modeling` | [`geometry`], [`topology`]          |
//! | [`meshing`]  | `monstertruck-meshing`  | [`mesh`], [`modeling`]              |
//! | [`solid`]    | `monstertruck-solid`, `monstertruck-fillet`, `monstertruck-healing` | [`meshing`] |
//! | [`assembly`] | `monstertruck-assembly` |                                 |
//! | [`step`]     | `monstertruck-step`     | [`modeling`]                      |
//! | [`gpu`]      | `monstertruck-gpu`      |                                 |
//! | [`render`]   | `monstertruck-render`   | [`gpu`], [`mesh`], [`meshing`]        |
//!
//! [`solid`] pulls three crates rather than one: the boolean kernel plus the
//! post-CSG, kernel-independent fillet and healing passes, which live in their
//! own crates so that a different boolean backend can be substituted without
//! duplicating them.
//!
//! ## Bundles
//!
//! | Bundle    | Features                                                               |
//! | --------- | ---------------------------------------------------------------------- |
//! | `default` | [`modeling`], [`meshing`]                                                  |
//! | `full`    | [`modeling`], [`meshing`], [`solid`], [`assembly`], [`step`], [`render`], `derive` |
//!
//! `rkyv` is orthogonal: it enables zero-copy serialisation in [`core`] and
//! [`topology`].
//!
//! ## Re-exported modules
//!
//! Each enabled feature exposes a top-level module:
//!
//! ```text
//! monstertruck::core        // always available
//! monstertruck::modeling    // with "modeling"
//! monstertruck::meshing     // with "meshing"
//! monstertruck::geometry    // with "geometry"
//! monstertruck::solid       // with "solid" -- the boolean kernel
//! monstertruck::fillet      // with "solid" -- post-CSG fillets
//! monstertruck::healing     // with "solid" -- post-CSG shape healing
//! monstertruck::step        // with "step"
//! ```

pub use monstertruck_core as core;

pub mod example_output;

#[cfg(feature = "traits")]
pub use monstertruck_traits as traits;

#[cfg(feature = "derive")]
pub use monstertruck_derive as derive;

#[cfg(feature = "geometry")]
pub use monstertruck_geometry as geometry;

#[cfg(feature = "topology")]
pub use monstertruck_topology as topology;

#[cfg(feature = "mesh")]
pub use monstertruck_mesh as mesh;

#[cfg(feature = "modeling")]
pub use monstertruck_modeling as modeling;

#[cfg(feature = "meshing")]
pub use monstertruck_meshing as meshing;

// `solid` is the BOOLEAN KERNEL and nothing else, so a drop-in external SSI
// backend exporting the same surface can replace the whole module and consumers
// need no code change. This is only possible because the post-CSG,
// kernel-independent passes (`fillet`, `healing`) are their own crates: while
// they lived inside `monstertruck-solid`, two kernels differed by more than
// their `ShapeOpsError`, and a glob facade would have exported two same-named
// error types.
#[cfg(feature = "solid")]
pub use monstertruck_solid as solid;

#[cfg(feature = "solid")]
pub use monstertruck_fillet as fillet;

#[cfg(feature = "solid")]
pub use monstertruck_healing as healing;

#[cfg(feature = "assembly")]
pub use monstertruck_assembly as assembly;

#[cfg(feature = "step")]
pub use monstertruck_io::step;

#[cfg(feature = "gpu")]
pub use monstertruck_gpu as gpu;

#[cfg(feature = "render")]
pub use monstertruck_render as render;
