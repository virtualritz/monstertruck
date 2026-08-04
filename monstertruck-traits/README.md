# `monstertruck-traits`

<!-- cargo-rdme start -->

Geometric trait definitions: `ParametricCurve`, `ParametricSurface`, `BoundedCurve`, `Invertible`, `Transformed`, and more.

## Examples

```rust
use monstertruck_traits::*;
use monstertruck_core::cgmath64::*;

// `range_tuple` comes from `BoundedCurve`, so the bound needs both traits.
fn arc_length<C: ParametricCurve<Point = Point3> + BoundedCurve>(
    curve: &C,
    steps: usize,
) -> f64 {
    let (t0, t1) = curve.range_tuple();
    let dt = (t1 - t0) / steps as f64;
    (0..steps)
        .map(|i| {
            let a = curve.evaluate(t0 + dt * i as f64);
            let b = curve.evaluate(t0 + dt * (i + 1) as f64);
            (b - a).magnitude()
        })
        .sum()
}
```

<!-- cargo-rdme end -->

> Forked from [`truck-geotrait`](https://crates.io/crates/truck-geotrait) v0.4.0 by [ricosjp](https://github.com/ricosjp/truck).

## License

Apache License 2.0
