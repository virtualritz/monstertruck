# `monstertruck-solid`

<!-- cargo-rdme start -->

Boolean operations for solids. Fillets live in `monstertruck-fillet` and shape
healing in `monstertruck-healing`; both are post-CSG and kernel-independent.

## Examples

```rust
use monstertruck_modeling::*;
use monstertruck_solid::or;

// Two unit cubes overlapping in a 0.5 cube at one corner.
let v = builder::vertex(Point3::origin());
let cube_a: Solid = builder::extrude(
    &builder::extrude(&builder::extrude(&v, Vector3::unit_x()), Vector3::unit_y()),
    Vector3::unit_z(),
);
let cube_b = builder::translated(&cube_a, Vector3::new(0.5, 0.5, 0.5));

// Boolean entry points are Result-shaped: `Ok(Solid)`, or a typed
// `ShapeOpsError` -- never a silent `None`.
// `monstertruck_modeling::*` brings its own 1-generic `Result` alias into
// scope, so leave the binding unannotated rather than shadowing it.
let union = or(&cube_a, &cube_b, 0.05);
assert!(union.is_ok());
```

<!-- cargo-rdme end -->

> Forked from [`truck-shapeops`](https://crates.io/crates/truck-shapeops) v0.4.0 by [ricosjp](https://github.com/ricosjp/truck).

## License

Apache License 2.0
