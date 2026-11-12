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
| IGES 4.0 to 5.3 | `iges` | no | read only, through cadmpeg -- see below |

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

## IGES: the conversion works, the decoder is the limit

Decoding is [`cadmpeg`](https://github.com/cadmpeg/cadmpeg)'s, which
monstertruck has no reason to reimplement. Turning the recovered geometry into
monstertruck's B-rep is ours, and it is written: see `cadmpeg` for what
reaches which surface variant, and why analytic carriers stay analytic.

Measured against `cadmpeg_ir::examples::unit_cube` -- cadmpeg's own document,
not a hand-built one -- a solid body converts to a shell of 8 vertices, 12
edges and 6 faces that `monstertruck_topology::Shell::extract` accepts and
reports CLOSED.

### What still stops a real file

**`cadmpeg-codec-iges` 0.6 decodes fixed-format ASCII IGES 4.0 to 5.3.** The
0.4 and 0.5 releases read 5.3 and then 5.1 to 5.3 only, and refused older
exports before the converter was reached.

Carriers this crate refuses on its own are refused BY NAME
(`Error::UnsupportedSurfaceKind`, `Error::UnsupportedCurveKind`): ellipses
and the other conics, composite curves, procedural carriers without an exact
solved cache, pole loops, and coedge-local edges. Periodic and unclamped NURBS
convert exactly since 0.6.
What the DECODER dropped reaches the caller in
`cadmpeg::Import::losses`.

### Nothing is ever silently empty

A caller cannot mistake any failure for "this file contained no geometry". An
empty document is `Error::NoGeometry`; a body that cannot be represented
fails the whole call and names itself, because a returned `Vec` cannot report
that one of its entries was left out. `Ok(vec![])` is unreachable, and
`tests/refuses_rather_than_returns_nothing.rs` pins that.

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
