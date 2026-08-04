# `monstertruck-healing`

<!-- cargo-rdme start -->

Shape healing for solids imported from other CAD systems.

The compressed-shell repair passes that turn a loaded B-rep into something a
boolean kernel can work with: closed-edge and closed-face splitting, seam
stripping, pass-through-edge dedup and shell orientation normalization.

These passes are POST-CSG and kernel-independent -- nothing here references a
boolean backend, which is why the crate sits below both the published
`monstertruck-solid` marching kernel and any external SSI boolean backend
rather than inside either.

<!-- cargo-rdme end -->

> Forked from [`truck-shapeops`](https://crates.io/crates/truck-shapeops) v0.4.0 by [ricosjp](https://github.com/ricosjp/truck).

## License

Apache License 2.0
