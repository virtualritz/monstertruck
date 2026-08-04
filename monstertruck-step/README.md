# `monstertruck-step`

<!-- cargo-rdme start -->

STEP file import and export.

## Examples

```rust
use monstertruck_step::load::{*, step_geometry::*};

// Parse a STEP file. Any `&[u8]` will do; this uses an in-repo fixture so
// the example is executed rather than merely type-checked.
let bytes = include_bytes!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../resources/step/occt-cube.step",
));
let table = Table::from_step_bytes(bytes).unwrap();

// Extract a shell and convert it to topology.
let step_shell = table.shell.values().next().unwrap();
let compressed = table.to_compressed_shell(step_shell).unwrap();
```

<!-- cargo-rdme end -->

> Forked from [`truck-stepio`](https://crates.io/crates/truck-stepio) v0.3.0 by [ricosjp](https://github.com/ricosjp/truck).

## License

Apache License 2.0
