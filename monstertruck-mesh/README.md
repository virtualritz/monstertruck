# `monstertruck-mesh`

<!-- cargo-rdme start -->

Polygon mesh data structures and algorithms.

## Examples

```rust
use monstertruck_mesh::*;

let positions = vec![
    Point3::new(0.0, 0.0, 0.0),
    Point3::new(1.0, 0.0, 0.0),
    Point3::new(0.0, 1.0, 0.0),
];
let faces = Faces::from_iter(&[[0, 1, 2]]);
let mesh = PolygonMesh::new(
    StandardAttributes { positions, ..Default::default() },
    faces,
);

assert_eq!(mesh.positions().len(), 3);
assert_eq!(mesh.tri_faces().len(), 1);
```

<!-- cargo-rdme end -->

> Forked from [`truck-polymesh`](https://crates.io/crates/truck-polymesh) v0.6.0 by [ricosjp](https://github.com/ricosjp/truck).

## License

Apache License 2.0
