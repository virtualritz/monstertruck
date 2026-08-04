# `monstertruck`

<!-- cargo-rdme start -->

**M**ultifarious **O**mnificence, **N**omenclature **S**tandardized,
**T**erminology **E**nhanced & **R**efactored **Truck** -- a **Ru**st
**C**ad **K**ernel.

A Rust CAD kernel -- B-rep modeling, NURBS curves and surfaces, meshing,
and GPU rendering. Forked from
[`ricosjp/truck`](https://github.com/ricosjp/truck) and renamed.

This is a meta-crate: it re-exports the workspace's `monstertruck-*`
sub-crates behind feature flags so downstream code can pull in exactly
the surface area it needs.

```toml
[dependencies]
monstertruck = { version = "0.3", features = ["full"] }
```

## Architecture

The kernel is split along the classic B-rep stack -- numerical primitives
at the bottom, topology and modeling in the middle, file I/O and rendering
at the top.

### Numerical core

- `core` -- `f64`-only linear algebra, tolerance helpers, bounding
  boxes, and the `derivatives` family used to
  evaluate B-spline/NURBS bases.
- `traits` -- the kernel's trait family
  (`ParametricCurve`,
  `ParametricSurface`,
  `SearchParameter`, etc.) together with the
  scalar-generic v2 successor under
  `v2`.
- `derive` -- proc-macros (`#[derive(StepFormat)]`, `#[derive(StepLength)]`)
  for the STEP back-end.

### Geometry and topology

- `geometry` -- concrete curves and surfaces: Bezier, B-spline, NURBS,
  conics, the `Processor` decorator,
  the new offset family
  (`OffsetCurve`,
  `OffsetSurface`,
  `NormalOffsetField`),
  and analytic specifieds such as
  `Sphere`.
- `topology` -- the B-rep graph: `Vertex`,
  `Edge`, `Wire`,
  `Face`, `Shell`,
  `Solid`, plus their `Compressed*` flattened forms
  used for serialisation and file I/O.

### Meshing

- `mesh` -- `PolygonMesh`, `StructuredMesh`, and the OBJ/STL
  serialisers.
- `meshing` -- tessellation algorithms that turn B-rep
  `Shell`/`Solid` into
  triangle soups for rendering or 3D printing.

### Modeling and boolean ops

- `modeling` -- the high-level builder
  (`builder`) for vertices, edges, wires,
  sweeps (`extrude`,
  `revolve`), tangent arcs, and primitive
  shapes (`cuboid`
  etc.).
- `solid` -- shape operators
  (`and`, `or`) for boolean combinations of
  solids. This is the boolean KERNEL only, so an external SSI backend
  exporting the same surface can stand in for it with no consumer code
  change.
- `fillet` -- rolling-ball edge fillets, a post-CSG pass.
- `healing` -- shape healing for solids imported from other CAD
  systems, also post-CSG.
- `assembly` -- DAG-structured assemblies (`Node`/`Edge`) that group
  parts and the rigid transforms between them.

### File I/O and rendering

- `step` -- STEP (ISO 10303-21) reader and writer for parts and
  assemblies.
- `gpu` -- `wgpu` device wrapper used by the renderer.
- `render` -- a small forward renderer that draws meshes produced by
  `meshing`.

## Example: subtract a cylinder from a box, then export STEP

Build a cuboid and a cylinder, subtract the cylinder via boolean `AND`
against its inverse, and emit the result as a STEP string.

```rust
use monstertruck::modeling::*;
use monstertruck::solid;
use monstertruck::step::save::{CompleteStepDisplay, StepHeaderDescriptor, StepModel};

// Unit cuboid centred on the origin.
let cube: Solid = primitive::cuboid(BoundingBox::from_iter([
    Point3::new(-0.5, -0.5, 0.0),
    Point3::new(0.5, 0.5, 1.0),
]));

// Cylinder of radius 0.3, axis along +Z, that pierces the cube top to bottom.
let seed = builder::vertex(Point3::new(0.3, 0.0, -0.1));
let rim = builder::revolve(
    &seed,
    Point3::new(0.0, 0.0, -0.1),
    Vector3::unit_z(),
    builder::SweepAngle::Closed,
    4,
);
let base = builder::try_attach_plane(&[rim])?;
let mut cylinder: Solid = builder::extrude(&base, Vector3::unit_z() * 1.2);

// Invert the cylinder so `and` performs a subtraction.
cylinder.not();
let punched = solid::and(&cube, &cylinder, 0.05)?;

// Flatten the topology and render as STEP.
let compressed = punched.compress();
let step = CompleteStepDisplay::new(
    StepModel::from(&compressed),
    StepHeaderDescriptor {
        organization_system: "monstertruck-doc".to_owned(),
        ..Default::default()
    },
)
.to_string();

assert!(step.starts_with("ISO-10303-21;"));
assert!(step.contains("MANIFOLD_SOLID_BREP"));
```

## Feature flags

Each sub-crate is gated behind a feature with the same name.

| Feature    | Crate                   | Also enables                    |
| ---------- | ----------------------- | ------------------------------- |
| _(always)_ | `monstertruck-core`     |                                 |
| `traits`   | `monstertruck-traits`   |                                 |
| `derive`   | `monstertruck-derive`   |                                 |
| `geometry` | `monstertruck-geometry` | `traits`                        |
| `topology` | `monstertruck-topology` |                                 |
| `mesh`     | `monstertruck-mesh`     | `traits`                        |
| `modeling` | `monstertruck-modeling` | `geometry`, `topology`          |
| `meshing`  | `monstertruck-meshing`  | `mesh`, `modeling`              |
| `solid`    | `monstertruck-solid`, `monstertruck-fillet`, `monstertruck-healing` | `meshing` |
| `assembly` | `monstertruck-assembly` |                                 |
| `step`     | `monstertruck-step`     | `modeling`                      |
| `gpu`      | `monstertruck-gpu`      |                                 |
| `render`   | `monstertruck-render`   | `gpu`, `mesh`, `meshing`        |

`solid` pulls three crates rather than one: the boolean kernel plus the
post-CSG, kernel-independent fillet and healing passes, which live in their
own crates so that a different boolean backend can be substituted without
duplicating them.

### Bundles

| Bundle    | Features                                                               |
| --------- | ---------------------------------------------------------------------- |
| `default` | `modeling`, `meshing`                                                  |
| `full`    | `modeling`, `meshing`, `solid`, `assembly`, `step`, `render`, `derive` |

`rkyv` is orthogonal: it enables zero-copy serialisation in `core` and
`topology`.

### Re-exported modules

Each enabled feature exposes a top-level module:

```text
monstertruck::core        // always available
monstertruck::modeling    // with "modeling"
monstertruck::meshing     // with "meshing"
monstertruck::geometry    // with "geometry"
monstertruck::solid       // with "solid" -- the boolean kernel
monstertruck::fillet      // with "solid" -- post-CSG fillets
monstertruck::healing     // with "solid" -- post-CSG shape healing
monstertruck::step        // with "step"
```

<!-- cargo-rdme end -->

## License

Apache License 2.0
