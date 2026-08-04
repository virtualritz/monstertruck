# `monstertruck-fillet`

<!-- cargo-rdme start -->

Fillet operations for `Shell` edges.

Provides rolling-ball fillet operations: single-edge fillets,
fillets with side face updates, and fillets along open or closed wire chains.
The `fillet_edges` function provides a high-level API that automatically
resolves face adjacency from edge IDs.

Fillets are POST-CSG and kernel-independent -- nothing here references a
boolean backend, which is why the crate sits below both the published
`monstertruck-solid` marching kernel and any external SSI boolean backend
rather than inside either.

<!-- cargo-rdme end -->

> Forked from [`truck-shapeops`](https://crates.io/crates/truck-shapeops) v0.4.0 by [ricosjp](https://github.com/ricosjp/truck).

## License

Apache License 2.0
