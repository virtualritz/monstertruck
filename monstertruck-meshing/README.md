# `monstertruck-meshing`

<!-- cargo-rdme start -->

Tessellation and meshing algorithms for B-rep shapes.

## Examples

```rust
use monstertruck_meshing::prelude::*;
use monstertruck_modeling::*;

// Build a cube and tessellate it
let v = builder::vertex(Point3::origin());
let cube: Solid = builder::extrude(
    &builder::extrude(&builder::extrude(&v, Vector3::unit_x()), Vector3::unit_y()),
    Vector3::unit_z(),
);

let mesh = cube.triangulation(0.01).to_polygon();
```

<!-- cargo-rdme end -->

> Forked from [`truck-meshalgo`](https://crates.io/crates/truck-meshalgo) v0.4.0 by [ricosjp](https://github.com/ricosjp/truck).

## License

Apache License 2.0
