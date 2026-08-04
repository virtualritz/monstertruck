# `monstertruck-derive`

<!-- cargo-rdme start -->

Derive macros for geometric traits. Re-exported by `monstertruck-traits`
(feature `"derive"`).

## Examples

The example is `ignore`d rather than run: it needs `monstertruck-traits`,
which DEPENDS on this crate, so this crate can never dev-depend on it to
link the doctest. These derives are exercised for real in
`monstertruck-traits/tests/derives.rs` (behind that crate's `derive` and
`polynomial` features), where the dependency direction works.

```rust
use monstertruck_traits::prelude::*;

/// An enum of curve types -- derive macros delegate trait methods
/// to the inner type via match arms.
#[derive(Clone, ParametricCurve, BoundedCurve)]
pub enum MyCurve {
    Line(Line<Point3>),
    Nurbs(NurbsCurve<Vector4>),
}

let curve: MyCurve = MyCurve::Line(/* ... */);
let pt = curve.evaluate(0.5); // dispatches to Line::evaluate
```

Users do not need to depend on this crate directly:

```toml
monstertruck-traits = { version = "0.3", features = ["derive"] }
```

<!-- cargo-rdme end -->

> Forked from [`truck-derivers`](https://crates.io/crates/truck-derivers) v0.1.0 by [ricosjp](https://github.com/ricosjp/truck).

## License

Apache License 2.0
