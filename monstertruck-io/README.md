# `monstertruck-io`

<!-- cargo-rdme start -->

Read and write CAD exchange formats as monstertruck B-rep.

One crate, one feature per format. A caller reaches every format monstertruck
supports through a single dependency instead of one crate per format, and
compiles only the formats it asks for.

## Formats

| format | feature | default | state |
| --- | --- | --- | --- |
| ISO 10303-21 (STEP) | `step` | yes | read and write, ours |
| IGES 5.3 | `iges` | no | **placeholder, see below** |

```rust
use monstertruck_io::step::load::{*, step_geometry::*};

let bytes = include_bytes!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../resources/step/occt-cube.step",
));
let table = Table::from_step_bytes(bytes).unwrap();
let step_shell = table.shell.values().next().unwrap();
let compressed = table.to_compressed_shell(step_shell).unwrap();
```

## STEP

`step` is monstertruck's own reader and writer, moved here from the retired
`monstertruck-step` crate. The API is unchanged -- `monstertruck_step::load`
became `step::load`, `monstertruck_step::save` became `step::save`.

It deliberately does **not** go through cadmpeg. It is measured against a
corpus, routes analytic surfaces onto closed forms so booleans stay exact,
and reports what a file lost. Measured 2026-08-04 on `occt-cylinder.step`:
monstertruck reads it correctly, and cadmpeg drops the whole
`MANIFOLD_SOLID_BREP` because one parameter-space `CIRCLE` fails to decode
(cadmpeg/cadmpeg#79). Writing STEP is ours outright.

## IGES, and why it refuses

The `iges` feature is a **placeholder**. It decodes a real file with a real
decoder -- [`cadmpeg`](https://github.com/cadmpeg/cadmpeg), which monstertruck
has no reason to reimplement -- but the last step, turning the recovered
geometry into monstertruck's B-rep, is unwritten. So it returns
`Error::Unimplemented` rather than an empty model, and an empty document
gets its own `Error::NoGeometry`. A caller therefore cannot mistake "not
written yet" for "this file contained no geometry". An importer that silently
returns nothing is worse than one that refuses, and
`tests/refuses_rather_than_returns_nothing.rs` fails the day it stops.

Adding a format is cheap because the cadmpeg codecs all decode into ONE
intermediate representation, `cadmpeg_ir::CadIr`: the per-format work is only
which decoder to call, and the part that carries the risk -- mapping recovered
geometry onto `monstertruck_topology` -- is shared, in `cadmpeg`.

<!-- cargo-rdme end -->

> The STEP reader and writer moved here from `monstertruck-step`, which is now
> a deprecated re-export. Forked from
> [`truck-stepio`](https://crates.io/crates/truck-stepio) v0.3.0 by
> [ricosjp](https://github.com/ricosjp/truck).

## License

Apache License 2.0
