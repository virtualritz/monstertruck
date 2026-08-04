# `monstertruck-topology`

<!-- cargo-rdme start -->

Topological data structures: vertices, edges, wires, faces, shells, and solids.

### Examples
The following sample code is a description of a topological tetrahedron as a solid model
by this package.
```rust
// Load topological structures with empty geometries.
monstertruck_topology::prelude!((), (), ());

// Create vertices. A tetrahedron has four vertices.
let v = Vertex::from_points(&[(); 4]);

// Create edges. Vertex is implemented the Copy trait.
let edge = [
    Edge::new(&v[0], &v[1], ()),
    Edge::new(&v[0], &v[2], ()),
    Edge::new(&v[0], &v[3], ()),
    Edge::new(&v[1], &v[2], ()),
    Edge::new(&v[1], &v[3], ()),
    Edge::new(&v[2], &v[3], ()),
];

// Create boundaries of faces as the wire.
// Edge is implemented the Copy trait.
let wire = vec![
    wire![&edge[0], &edge[3], &edge[1].inverse()],
    wire![&edge[1], &edge[5], &edge[2].inverse()],
    wire![&edge[2], &edge[4].inverse(), &edge[0].inverse()],
    wire![&edge[3], &edge[5], &edge[4].inverse()],
];

// Create faces by the boundary wires.
// The boundary of face must be simple and closed.
let mut face: Vec<Face> = wire.into_iter().map(|wire| Face::new(vec![wire], ())).collect();
face[3].invert();

// Create shell of faces. Shell can be created by `Vec<Face>``.
let shell: Shell = face.into();

// Create a tetrahedron solid by the boundary shell.
// The boundaries of a solid must be closed and oriented.
let solid = Solid::new(vec![shell]);
```
### Elements and containers
Main structures in `monstertruck_topology` consist 4 topological elements and 2 topological containers.
#### Topological elements
The following structures are topological elements.

* [`Vertex`](./struct.Vertex.html)
* [`Edge`](./struct.Edge.html)
* [`Face`](./struct.Face.html)
* [`Solid`](./struct.Solid.html)

Except `Solid`, each topological element has a unique `id` for each instance.
In higher-level packages, by mapping this `id` to geometric information, you can draw a solid shape.
#### Topological containers
The following structures are topological container.

* [`Wire`](./struct.Wire.html)
* [`Shell`](./struct.Shell.html)

The entities of `Wire` and `Shell` are `std::collections::VecDeque<Edge>` and `std::vec::Vec<Face>`,
respectively, and many methods inherited by `Deref` and `DerefMut`.
These containers are used for creating higher-dimensional topological elements and checked the
regularity (e.g. connectivity, closedness, and so on) before creating these elements.
### Features
* `nightly` -- Use features available only in a `nightly` toolchain.
* `rclite` -- Use of `rclite::Arc` instead of `std::syn::Arc`. The latter
  uses more memory and is potentially slower than the former. On by default.

<!-- cargo-rdme end -->

> Forked from [`truck-topology`](https://crates.io/crates/truck-topology) v0.6.0 by [ricosjp](https://github.com/ricosjp/truck).

## License

Apache License 2.0
