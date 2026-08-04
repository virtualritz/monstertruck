# Change Log

The version is of the bottom crate `monstertruck-render`.

## Unreleased

- **READMEs are now generated from crate docs by `cargo rdme`**, verified in CI. The crate-level `//!` docs are the single source of truth; attribution and license lines sit outside the `cargo-rdme` markers and are preserved. New `just readme` regenerates, `just readme-check` verifies, and `ci` runs the check. This turned four rotted README examples into compiled doctests and fixed them: `monstertruck-topology` called the removed `Vertex::news`, `monstertruck-step` imported the pre-rename `monstertruck_step::in` module, `monstertruck-traits` was missing a `BoundedCurve` bound, and `monstertruck-solid` bound a `Result` to `Option` while demonstrating a cube union in the one configuration that fails. `monstertruck-fillet` and `monstertruck-healing` gained the READMEs they were missing.
- Removed the `readme-generator` crate. It shelled out to `cargo readme`, which is no longer installed, and truncated each `README.md` *before* invoking it -- running it would have blanked 11 READMEs and then panicked.
- **Naming: `row_curve`/`column_curve` are now `curve_u`/`curve_v`.** The row/column vocabulary was crossed with respect to the parameter that actually varies -- the *row* curve varies `u`, the *column* curve varies `v` -- which had already produced four mislabeled doc comments in the crates themselves. The index parameters are now `index_v`/`index_u`, naming what gets pinned rather than what varies. The upstream `truck` spellings remain as `#[deprecated]` forwarders.
- **Naming: `u_*`/`v_*` accessors are now `*_u`/`*_v`.** The convention across these crates is `<property>_<direction>` -- `derivative_u`, `knot_vector_u`, `cut_u`, `add_knot_u`, `remove_knot_u` -- but a handful of accessors used the prefix form, including two on the same trait as `derivative_u`. `ParametricSurface::u_period`/`v_period` are now `period_u`/`period_v`, and `Plane::u_axis`/`v_axis` are now `axis_u`/`axis_v`. The old names remain as `#[deprecated]` forwarders, so callers keep compiling; only code that *implements* `ParametricSurface` by hand must move to the new method names. The scalar-generic `v2` trait family already used the postfix form, so this also removes the divergence between the two families.
- **New crate `monstertruck-healing`**: the shape-healing passes for solids imported from other CAD systems move out of `monstertruck-solid` into their own crate (`split_closed_edges`, `split_closed_faces`, `split_pass_through_edges`, `split_seam_faces`, `strip_seam_edges`). They are post-CSG and kernel-independent, so they no longer belong inside a boolean kernel.
- **New crate `monstertruck-fillet`**: rolling-ball fillet and chamfer operations on shell edges move out of `monstertruck-solid` for the same reason. `monstertruck-modeling`'s `fillet` feature now re-exports it; the `monstertruck` facade exposes it as `monstertruck::fillet` under the `solid` feature.
- `monstertruck-solid` is now the boolean kernel and nothing else, and is fully self-contained: it carries the truck-derived polyline-marching pipeline as its default backend (`marching-ssi`, on by default) and no heavy linear-algebra or interval-arithmetic dependencies.
- Under `monstertruck-solid` `--no-default-features` no boolean backend is compiled; the boolean entry points return a typed `ShapeOpsError::NoBackend` instead of panicking. An external SSI backend exporting the same surface can stand in for the whole `solid` module.
- **`monstertruck-traits`**: `parameter_division` is now bounded. New `parameter_division_with_budget`, `try_parameter_division`, `BudgetedDivision` and the typed `DivisionTruncated` outcome replace the previously unbounded recursion, which could subdivide without limit on surfaces whose closed-form inverse was discarded. Also new: `division_work`/`division_totals`/`division_max_cells` counters for measuring subdivision cost.
- **`monstertruck-step` load path**: new `from_step_bytes` entry point; a new structured load report (`load::report`) that names what a file lost and why instead of dropping it silently; UV-domain clamping for STEP surfaces with placeholder parameter ranges; analytic routing for `CYLINDRICAL_SURFACE`, `CONICAL_SURFACE`, `SPHERICAL_SURFACE` and trimmed surfaces of revolution onto closed-form geometry rather than a rational B-spline degrade.
- Fixed: `SURFACE_OF_LINEAR_EXTRUSION` conversion built a **transposed** control net.
- Fixed: STEP loads could produce shells whose faces did not share an orientation, which made a divergence-theorem volume come out negative. `monstertruck-healing` gains a signed-volume oracle over the in-repo analytic fixtures.
- Fixed: the surface projector no longer returns a non-finite `(u, v)`, and no longer queries placeholder parameter domains that cannot answer.
- Fixed: a fixed `1e-6` tolerance in surface footpoint search was rejecting footpoints the caller had certified exact.
- **`monstertruck-meshing`**: tessellation gains a strict mode and structured face-drop diagnostics, so a dropped face becomes a typed refusal or a logged warning instead of a silently smaller mesh. Adds a `log` dependency (a zero-cost facade when no logger is installed).
- Empty-parameter-polyline and degenerate-torus inputs now return typed outcomes instead of panicking.
- Reference the adjacent face's surface entity from `SURFACE_CURVE` edge associations instead of re-emitting it, shrinking exported STEP files.
- Save modeling cylinders as an analytic `CYLINDRICAL_SURFACE` in STEP output instead of a rational B-spline degrade.
- Make the STEP length unit and `distance_accuracy_value` configurable via `StepMeasurementContext`.
- In the README, we clarified that the subtitle is the origin of the name "truck," and changed all instances of the term in the main text to `truck`.
- Get more precise part attributions from `Product` and `NextAssemblyUsageOccurrence`.
- Add the variable `division` to `monstertruck_modeling::builder::rsweep`.
- Renew DAG structure.
- Fix spell and replace `Fn` to `FnMut`.
- Read assembly from step file.
- Implement assembly structure handler `monstertruck-assembly`.
- Downgrade `cargo` for `cargo doc`. cf: https://github.com/rust-lang/rust/issues/148431
- Update docker container and `Makefile.toml` for `gpu-test`.
- Update docker container `gpu-test`
- Fix step output of `CylindricalSurface`.
- Remove `Arc` from the members of `DeviceHandler`.
- Implement `border_wires` for `Face`.
- Implement `From` and `ToSameGeometry` from `ExtrudeCurve<Line<Point3>, Vector3>` to `Plane`.
- Fix comparative phrasing.
- Fix `SceneInfo` in `polygon.wgsl`.
- Upgrade wgpu v26.
- Approximation of `RbfSurface` by `ApproxFilletSurface`.
- Align mesh aspects of general surfaces tessellation.
- Refactoring: `intersection_curve` and `Homogeneous`.
- Implement `CurveDers` and `SurfaceDers`.
- Loosened `cut_random_test` requirements.
- Higher order derivations.
- Renew `Camera`.
- Constant allocation for faster B-spline basis function.
- New implementation for `search_parameter`.
- Add `RbfSurface`.
- Add `prop_assert_near` for `proptest` integration.
- Primitives: rect, circle, and cuboid.
- The zoom of the parallel camera has been made to work.
- Minor change.
- Fix some typos.
- Saving memory of `put_together_same_attrs`.
- Closed mesh with `robust_triangluation`.
- Implement `Transformed<Matrix4>` for `PolygonMesh`.
- Fix some step output.
- `cargo upgrade -i`
- Create `CYLINDRICAL_SURFACE` by `builder::rsweep`.
- Step output for specified revoluted surface.
- Remove `println` for debugging.
- Generalize `monstertruck_modeling::builder` for apply step parsed geometries.
- Review of the specifications for `IntersectionCurve`.
- Fix STEP header description.
- Fix some typos.
- Implement `BSplineCurve::interpole`.
- Implement `search_intersection_parameter` between surface and curve.
- Add macros: `wire` and `shell`.
- Strict derivation and `search_parameter` of `IntersectionCurve`.
- Prototyping for fillet surface with NURBS geometry.
- Implement abstract newton method.
- Minor correction of `double_projection`.
- Update algorithm of `double_projection`.
- More improve of `monstertruck_traits::algo::surface::search_parameter`.
- Simplify `monstertruck_traits::algo::surface::search_parameter`.
- Add the macro `monstertruck_topology::prelude!`.

### Latest `cargo upgrade`

2026-02-12

## v0.6

### Additional APIs

- `monstertruck_step::in` has been released!
  - Parse some geometries: B-spline, NURBS, elementary geometries, and so on.
  - Parse topologies: shell and solid.
  - JS wrappers.
- Implement `robust_triangulation`, trimming meshes by `SearchNearestParameter`.
- Output meshes by vtk formats.
- Split closed edges and faces, loaded from STEP (generated by other CAD systems).
- Calculate volume and center of the gravity of `PolygonMesh`.
- Derive macros for implementing `StepLength` and `DisplayByStep`.
- `area` and `include` function for a domain with several polyline boundaries.

### Updated APIs

- Add "periodic" identifier to `ParametricCurve` and `ParametricSurface`.
- Remove the `Invertible` constraint from tessellating traits.
- Features has been set up to use each module in `monstertruck-meshing` separately.
- Non-bounded parameter ranges has been supported. Updates `ParametricXXX` and `BoundedXXX`.
- Derive macros in `monstertruck-derive` are supported for cases with generics.
- Implement `SearchNearestParameter` for `Processor`.
- Expanded coverage of tessellation API.
  - Enabled meshing when the boundary is not closed in the parameter space.
  - Add tessellate test with ABC Dataset.
- Improve `put_together_each_attrs`.
  - Add an argument to `put_together_each_attrs` to specify the tolerance.
  - Transitive clustering instead of spatial partitioning by rounding
- Improve `Shell::face_adjacency`: Common edges are now also retrieved.

### Bug fix

- Change the precision of floating point numbers when outputting STEP files.
- Updates `SearchNearestParameter` for `RevolutedCurve`.
- Fix a bug on partial `rsweep` with a negative angle.
- Add a private function `spade_round` for fixing insert error.

### Internal Improvements

- Replace `Mutex` and `Arc` more faster and compact mem.
- Refactor and renew test for `monstertruck_modeling::geom_impl` by `proptest`.
- Add tests for traits in `monstertruck_modeling::topo_traits`.
- Implementation for closed surface tessellation.
- Implelment `AsRef`, `Borrow`, and `Extend` for `Wire` and `Shell`.

### Misc

- Changed some naming conventions to Rust standards.
  - Make some struct naming canonical. ex: NURBSCurve -> NurbsCurve.
  - Remove `get_` prefix from `Vertex::get_point`, `Edge::get_curve`, and `Face::get_surface`.
- Put `monstertruck_geometry::prelude` for resolve multiple re-export.
- Tutorial for v0.6 series has been released.

## v0.5

### Additional APIs

- derive macros for geometric traits [`truck-geoderive`](truck-geoderive)
- step output of open shell, worlds including several models, and `IntersectionCurve`
- parallel iterators for topological structures
- direct tessellation of `CompressedShell` and `CompressedSolid`
- direct serialization for topological data structures.
- cubic B-spline approximation
- `builder::try_wire_homotopy`
- `Solid::cut_face_by_edge`
- `Face::edge_iter` and `Face::vertex_iter`
- `IntersectionCurve` between `Plane`s can now be converted to `Line`.
- `Camera::ray`
- `EntryMap`

### Updated APIs

- `MeshableShape::triangulation`
- the Euler operations
- `Face::cut_by_edge`
- Refactoring `Search(Nearest)Parameter`.

### Bug fix

- The orientation of the normal of `builder::try_attach_plane`.
- `Shell::singular_vertices`
- binary STL output of `PolygonMesh`

### Internal Improvements

- Data integrity check during deserialization of `KnotVec`, `BSplineCurve`, and all structs constructed by `try_new`.
- Improve meshing algorithm by parallelization.
- Intersection curve with B-spline leader.
- Implement some geometric traits for `TrimmedCurve`, `UnitHyperbola` and `UnitParabola`.
- Use Line in modeling and simplify output shape of tsweep.

### Misc

- Make `TextureFormat` of surfaces `BrgaU8norm`.
- Add an example with several boundaries.
- Updates `wgpu` to `v0.14`
- Updates `spade` to `v2`.
- Change the profile of `monstertruck-wasm` and remove dependencies to `wee_alloc`.

## v0.4

- The first version of `monstertruck-step` has been released! One can output shapes modeled by `monstertruck-modeling`.
- WGSL utility `math.wgsl` has been released! One can calculate invert matrices and rotation matrices.
- The processing related to linear algebra has been isolated from `monstertruck-core` to [`matext4cgmath`](https://crates.io/crates/matext4cgmath).
- New mesh filter `Subdivision::loop_subdivision` was implemented in `monstertruck-meshing`!
- In `monstertruck-traits`, the trait `ParametricCurve` is decomposed into `ParametricCurve` and `BoundedCurve`.
- The method `swap_vertex` has been added to `WireFrameInstance`.
- Geometric traits has been derived to `Box`.
- Some specified geometries has been added for STEP I/O
- Comparing `BoundingBox` by inclusion relationship.
- In order to make meshing reproducible, we decided to implement random perturbations by means of a deterministic hash function.
- Some lints has been added.

## v0.3

- Specified surface for STEP I/O and modeling revolved sphere and cone.
  - In `monstertruck-core`, the trait `Surface` is decomposed into `ParametricSurface`, `BoundedSurface`, `IncludeCurve` and `Invertible`.
  - In `monstertruck-geometry`, specified surface, `Plane` and `Sphere`, and some decorators are prepared.
- STL handling module `stl` in `monstertruck-mesh`.
- In `monstertruck-render`, wireframe for polygon.
  - Abort traits `Shape` and `Polygon`, and add new traits `IntoInstance` and `TryIntoInstance`.
- Applied wgpu v0.11 and made all shaders WGSL, including shaders for test. Now, all dependence on cmake has been removed!
  - The sample code `glsl-sandbox` becomes `wgsl-sandbox`. You can easily experience WGSL shading.
- Split `monstertruck-core::geom_trait` into `monstertruck-traits` and added some algorithms `algo`. Some methods in curves and surfaces were standardized.
- Added a new crate `monstertruck-meshing`. Moved the polygon processing algorithm from polymesh to meshalgo.
- Added a new CAD meshing algorithm. Meshing trimmed surfaces. The same edge is made into the same polyline. A solid is made into a closed polygon.
- Added some meshing algorithms, including mesh collision.
- `ShapeInstance` has been removed. Tessellation should be done in advance by `monstertruck-meshing` when drawing the modeled shape.
- `BSplineCurve<Point3>` was made to be `ParametricCurve3D`. Conflicts related to methods `subs` have been resolved.
- Added a new crate `monstertruck-solid`, which provides solid boolean operator functions: `and` and `or`.
- Added a new crate `monstertruck-wasm`, which provides wasm bindings of CAD APIs. (not released to crates.io)

## v0.2

### v0.2.1

- a small behavior change: [`NormalFilters::add_smooth_normals`](https://docs.rs/monstertruck-mesh/0.2.1/monstertruck_mesh/prelude/trait.NormalFilters.html#tymethod.add_smooth_normals).
- fix a bug: [`Splitting::into_components`](https://docs.rs/monstertruck-mesh/0.2.1/monstertruck_mesh/prelude/trait.Splitting.html#tymethod.into_components).
- an internal change: [`RenderID::gen`](https://docs.rs/monstertruck-gpu/0.2.1/monstertruck_gpu/struct.RenderID.html#method.gen).

### v0.2.0

- made `monstertruck-mesh` stable (well-tested and safety)
  - The member variables of [`PolygonMesh`](https://docs.rs/monstertruck-mesh/0.2.0/monstertruck_mesh/struct.PolygonMesh.html) becomes private.
    - Destructive changes to the mesh are provided by [`PolygonMeshEditor`](https://docs.rs/monstertruck-mesh/0.2.0/monstertruck_mesh/polygon_mesh/struct.PolygonMeshEditor.html), which checks the regularity of the mesh at dropped time.
  - Mesh handling algorithms are now a public API.
    - The hidden structure `MeshHandler` was abolished and algorithms are managed as traits.
    - You can use them by importing [`monstertruck_mesh::prelude::*`](https://docs.rs/monstertruck-mesh/0.2.0/monstertruck_mesh/prelude/index.html).
- improved `monstertruck-render` for higher performance and better usability
  - Wire frame rendering for shapes are now available.
    - One can create [`WireFrameInstance`](https://docs.rs/monstertruck-render/0.2.0/monstertruck_render/struct.WireFrameInstance.html) by [`InstanceCreator::create_wire_frame_instance`](https://docs.rs/monstertruck-render/0.2.0/monstertruck_render/struct.InstanceCreator.html#method.create_wire_frame_instance).
    - Try to run `cargo run --example wireframe`.
  - [`InstanceDescriptor`](https://docs.rs/monstertruck-render/0.1.5/monstertruck_render/struct.InstanceDescriptor.html) is separated into [`PolygonInstanceDescriptor`](https://docs.rs/monstertruck-render/0.2.0/monstertruck_render/struct.PolygonInstanceDescriptor.html) and [`ShapeInstanceDescriptor`](https://docs.rs/monstertruck-render/0.2.0/monstertruck_render/struct.ShapeInstanceDescriptor.html).
    - One can specify the precision of meshing faces by `ShapeInstanceDescriptor::mesh_precision`.
    - The old `InstanceDescriptor` is renamed to [`InstanceState`](https://docs.rs/monstertruck-render/0.2.0/monstertruck_render/struct.InstanceState.html).
    - The descriptor for wire frames is [`WireFrameInstanceDescriptor`](https://docs.rs/monstertruck-render/0.2.0/monstertruck_render/struct.WireFrameInstanceDescriptor.html).
  - added [`InstanceCreator`](https://docs.rs/monstertruck-render/0.2.0/monstertruck_render/struct.InstanceCreator.html) for generating instances.
    - `InstanceCreator` has pre-compiled shader modules as member variables.
    - [`CreateInstance`](https://docs.rs/monstertruck-render/0.1.5/monstertruck_render/trait.CreateInstance.html) for `Scene` is abolished.
    - `InstanceCreator` is created by [`Scene::instance_creator`](https://docs.rs/monstertruck-render/0.2.0/monstertruck_render/trait.CreatorCreator.html#tymethod.instance_creator).
  - Face-wise rendering of shape is abolished.
    - Now, `ShapeInstance` is one `Rendered` struct.
    - [`RenderFace`](https://docs.rs/monstertruck-render/0.1.5/monstertruck_render/struct.RenderFace.html) was abolished.
  - abolished implementations `Clone` for `*Instance`. Use `*Instance::clone_instance`.
  - The texture of `InstanceState` was changed `wgpu::Texture` from `image::DynamicImage`.  
    One can generate `Texture` from `DynamicImage` by [`InstanceCreator::create_texture`](https://docs.rs/monstertruck-render/0.2.0/monstertruck_render/struct.InstanceCreator.html#method.create_texture).
- added inherit methods of `monstertruck_geometry::NURBSSurface` from `BSplineSurface`.
- added a feature `serde` to `cgmath` at `monstertruck-core`.
  - remove the explicit dependency to `cgmath` from `monstertruck-mesh`.
  - plans to add `nalgebra` as an alternative backend (unreleased in this version).
- abolished [`monstertruck_gpu::RenderID::default`](https://docs.rs/monstertruck-gpu/0.1.0/monstertruck_gpu/struct.RenderID.html#impl-Default) and added [`RenderID::gen`](https://docs.rs/monstertruck-gpu/0.2.0/monstertruck_gpu/struct.RenderID.html#method.gen).
- added [`Error`](https://docs.rs/monstertruck-modeling/0.2.1/monstertruck_modeling/errors/enum.Error.html) to `monstertruck_modeling`.
- made [`monstertruck_topology::CompressedShell`](https://docs.rs/monstertruck-topology/0.2.0/monstertruck_topology/struct.CompressedShell.html) public API and added [`monstertruck_topology::CompressedSolid`](https://docs.rs/monstertruck-topology/0.2.0/monstertruck_topology/struct.CompressedSolid.html).

## v0.1

### v0.1.5

- changed a behavior of [`monstertruck_topology::try_add_boundary`](https://docs.rs/monstertruck-topology/0.1.1/monstertruck_topology/struct.Face.html#method.try_add_boundary) and [`monstertruck_topology::add_boundary`](https://docs.rs/monstertruck-topology/0.1.1/monstertruck_topology/struct.Face.html#method.add_boundary).
  - flip the boundary over when adding a boundary to a face with a flipped orientation
  - renew the id of the face which was added boundary

### v0.1.4

- add a method: `monstertruck_render::*Instance::clone_instance`
- `Clone::clone for *Instance` is deprecated, and will be abolished in v0.2.

### v0.1.3

- fixed two bugs
  - [`monstertruck_modeling::builder::homotopy`](https://docs.rs/monstertruck-modeling/0.1.3/monstertruck_modeling/builder/fn.homotopy.html), the vertices were in the wrong order.
  - [`monstertruck_modeling::Mapped for Shell`](https://docs.rs/monstertruck-modeling/0.1.3/monstertruck_modeling/topo_traits/trait.Mapped.html#impl-Mapped%3CP%2C%20C%2C%20S%3E-for-Shell%3CP%2C%20C%2C%20S%3E), the orientation of surface was wrong.

### v0.1.2

- fixed a bug: [`monstertruck_modeling::builder::try_attach_plane`](https://docs.rs/monstertruck-modeling/0.1.2/monstertruck_modeling/builder/fn.try_attach_plane.html), the orientation of plane was incorrect.

### v0.1.1

- fixed a bug: [`monstertruck_modeling::builder::rsweep`](https://docs.rs/monstertruck-modeling/0.1.1/monstertruck_modeling/builder/fn.rsweep.html), the boundary was incorrect.

### v0.1.0

- first version
