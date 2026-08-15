# `monstertruck`

[![crates.io](https://img.shields.io/crates/v/monstertruck.svg)](https://crates.io/crates/monstertruck)
[![docs.rs](https://img.shields.io/docsrs/monstertruck)](https://docs.rs/monstertruck)
[![CI](https://img.shields.io/github/actions/workflow/status/virtualritz/monstertruck/ci.yml?branch=master&label=ci)](https://github.com/virtualritz/monstertruck/actions/workflows/ci.yml)
[![License: Apache-2.0](https://img.shields.io/crates/l/monstertruck.svg)](#license)

**M**ultifarious **O**mnificence, **N**omenclature **S**tandardized, **T**erminology **E**nhanced & **R**efactored **Truck** -- a **Ru**st **C**ad **K**ernel.

## Contents

- [Overview](#overview)
- [Why Was This Forked?](#why-was-this-forked)
  - [Improvements Since the Fork](#improvements-since-the-fork)
  - [Keeping in Sync with `truck`](#keeping-in-sync-with-truck)
- [Usage](#usage)
  - [Running the Examples](#running-the-examples)
- [Architecture & Crate Ecosystem](#architecture--crate-ecosystem)
  - [Core & Geometry](#core--geometry)
  - [Topology & Modeling](#topology--modeling)
  - [Meshing & Rendering](#meshing--rendering)
  - [I/O & Bindings](#io--bindings)
- [Dependency Graph](#dependency-graph)
- [License](#license)

## Overview

`monstertruck` is an open-source, Rust-based shape processing kernel. It is a fortified, feature-expanded fork of the original [`truck`](https://github.com/ricosjp/truck) project.

## Why Was This Forked?

Getting PRs accepted upstream was proving to be a challenge, so we spun up `monstertruck` to keep development moving a tad faster.

This fork exists to accomplish two main goals:

1. **Supercharge the functionality:** We are adding and enhancing features, tools, and operations that go beyond the original scope (hence the _Multifarious Omnificence_). This includes merging `truck` PRs that we deem useful (but you are welcome to open PRs against `monstertruck` directly, ofc!).

2. **Fix the ergonomics:** The original codebase suffered from unconventional phrasing, non-idiomatic naming conventions, and occasionally confusing translations.
   We have overhauled the project using idiomatic Rust naming conventions and standard CAD terminology.
   Our goal is to make the codebase inclusive, readable, and accessible -- whether you are a non-native English speaker or a seasoned CAD veteran.

### Improvements Since the Fork

Per-crate detail and porting verdicts live in [`TRUCK-PARITY.md`](TRUCK-PARITY.md); upstream commits we hand-ported include SHAs in their commit bodies for attribution.

**Workspace Modernization**
- All crates renamed `truck-*` -> `monstertruck-*`; `truck-platform` -> `monstertruck-gpu`, `truck-stepio/src/{in,out}` -> `monstertruck-io/src/step/{load,save}` (via `monstertruck-step`, since retired), `truck-shapeops` -> `monstertruck-solid`.
- Rust edition 2024, `wgpu` 29, `rand` 0.10, `criterion` 0.8, `gloo` 0.12; `web_time::Instant` for `wasm`.
- `vtk` dropped from default features over [RUSTSEC-2026-0041](https://rustsec.org/advisories/RUSTSEC-2026-0041.html); opt-in only.
- Workspace `Cargo.toml` consolidates shared deps; `just` replaces `cargo-make`; GitHub Actions replaces GitLab CI; `fmt --check` runs on nightly so `rustfmt.toml`'s unstable options actually apply.

**API Ergonomics & Naming**
- Result-shaped boolean ops: `solid::and`/`or`/`difference`/`symmetric_difference` return `Result<Solid, ShapeOpsError>`.
- `LoadError` thiserror enum on the STEP loader; `Table::from_step` returns `Result`.
- Parameter-space markers `D1`/`D2` -> `CurveParameter`/`SurfaceParameter`; `Univariate*`/`Bivariate*ScalarFunction` -> `CurveScalarFunction`/`SurfaceScalarFunction`; `KnotVec` -> `KnotVector`; `BSpline*` -> `Bspline*`.
- Public names de-abbreviated: `attrs()` -> `attributes()`, `PartAttrs` -> `PartAttributes`, `DisplayByStep` -> `StepFormat`, `assy` -> `assembly`, `rbf_surface` -> `rolling_ball_fillet`, `af_surface` -> `approximate_fillet_surface`, `interpole` -> `interpolate`.
- Accessors follow `<property>_<direction>`, matching `derivative_u`: `u_period`/`v_period` -> `period_u`/`period_v`, `Plane::u_axis`/`v_axis` -> `axis_u`/`axis_v`, `row_curve`/`column_curve` -> `curve_u`/`curve_v` (the row/column pair was crossed -- the *row* curve varies `u`).
- Every rename ships a deprecated alias so upstream-style code still compiles.

**Geometry Correctness**
- `Sphere::search_nearest_parameter` guards: `acos` clamp, exact-pole `0/0` singularity, `point == center` singularity.
- `parameter_range()` fix for non-clamped B-splines.
- Surface `parameter_division` recursion guard + decorrelated jitter (partial port of upstream [`7b1f4171`](https://github.com/ricosjp/truck/commit/7b1f4171)).
- `UnitCircle::search_{nearest_}parameter` honors `hint` across the period (upstream [`f563ae53`](https://github.com/ricosjp/truck/commit/f563ae53) + [`86e4ed75`](https://github.com/ricosjp/truck/commit/86e4ed75)).
- STEP `Axis2Placement3d` guards parallel `axis`/`ref_direction`; revolved-line-to-cylinder conversion drops the spurious inversion (upstream [`524f5f53`](https://github.com/ricosjp/truck/commit/524f5f53)); rational trim boundaries preserved through the load path; inverted-processor sample alignment.

**Meshing**
- Triangulation/tessellation pipeline rewritten; CDT trim-constraint handling rebuilt across [`b60b1604`](https://github.com/virtualritz/monstertruck/commit/b60b1604)/[`46b21f9f`](https://github.com/virtualritz/monstertruck/commit/46b21f9f)/[`f35d3b6d`](https://github.com/virtualritz/monstertruck/commit/f35d3b6d)/[`7c5ce2d2`](https://github.com/virtualritz/monstertruck/commit/7c5ce2d2) (skip conflicting, avoid invalid, preserve split, split through vertices).
- `PolyBoundary::include` gets an AABB early reject; double tessellation removed in `step-to-mesh`.
- Tessellation benchmark example + baseline log for regression tracking.

**Boolean Operations**
- Reverted upstream's [`700138cb`](https://github.com/virtualritz/monstertruck/commit/700138cb)-equivalent boolean rewrite after bisect confirmed it regressed `punched_cube` and `adjacent_cubes_or` (output shells came back `Oriented`, not `Closed`); we keep the upstream-derived single-ray algorithm wrapped in our `Result` layer.
- New `strip_seam_edges` healing pass: when a wire visits the same edge twice with opposite orientations (the canonical STEP cylinder/cone seam pattern), the pass cuts at the seam edge and emits two simple wires on the same face. Fixes `NotSimpleWire` extraction failures on `abc-0008.step`, `occt-cylinder.step`, `occt-cone.step`. No upstream equivalent.

**New Capabilities**
- Offset geometry: `OffsetCurve`, `OffsetSurface`, `NormalOffsetField`, `CurveScalarFunction`, `SurfaceScalarFunction` (renamed from upstream's `Offset`/`NormalField`/`ScalarFunctionD*`; upstream [`9031e6dd`](https://github.com/ricosjp/truck/commit/9031e6dd)).
- Assembly STEP output: `StepDesign`, `MatrixAsAxis`, full `save::assembly` module (upstream `213-assy-step-output`).
- Tangent-based circular arc construction in `monstertruck-modeling`: `CircularArcConstraint::{ThroughPoint, StartTangent}`, `try_circle_arc_by_start_tangent` (renamed from upstream `ArcConstraint`/`circle_arc_by_tangent0`; upstream [`993e156c`](https://github.com/ricosjp/truck/commit/993e156c)).
- Fillet engine rewrite: per-edge radii, variable-radius open wires, multi-chain + chamfer, `Ridge` and `Custom` profile modes, robust topology surgery, degenerate-edge rejection, `IntersectionCurve` support.
- T-spline / T-NURCC promoted to first-class surface type with `BsplineSurface` conversion, curvature-based adaptive refinement, and hot-path optimization (lock-free `Tmesh::subs()`, flat-array layout for `analytical_der_mn()`).
- Scalar-generic `v2` trait family (`CurveParameter<T>`/`SurfaceParameter<T>`, `SearchParameter<v2::D2<T>>`, etc.) -- no upstream equivalent; default scalar still `f64`.
- `SurfaceDerivatives::absolute_derivatives` + `combinatorial_derivative(s)` ported from upstream's `truck-base::ders`, backing the offset surface family.
- `BasisWindow` active-window B-spline basis evaluation (upstream [`77e25635`](https://github.com/ricosjp/truck/commit/77e25635)), reimplemented with `SmallVec`; both `BsplineCurve` and `BsplineSurface` only touch active control points.
- STEP face preview tool at [`monstertruck-io/examples/preview-step-face.rs`](monstertruck-io/examples/preview-step-face.rs) for diagnostic visualization -- canonical replacement for ad-hoc `eprintln!`-in-`loops_store` debugging; see [AGENTS.md](AGENTS.md#visual-debugging-for-meshingtrim-bugs).

**I/O and Exchange Formats**
- All exchange formats consolidated into [`monstertruck-io`](monstertruck-io/), one feature per format, so a caller reaches every format through a single dependency and compiles only what it asks for. `monstertruck-step` remains as a deprecated re-export, so an existing `monstertruck-step = "0.3"` requirement keeps resolving.
- ISO 10303-21 now parses through [`step-p21`](https://crates.io/crates/step-p21), our published fork of [`ruststep`](https://github.com/ricosjp/ruststep). Two syntax fixes real CAD exports need had sat unreleased on ruststep master for four years: `''` as an escaped apostrophe (which imperial CAD emits as inch marks in thread callouts) and `()` as an empty aggregate. A `[patch.crates-io]` fixes your own build but never your dependents, so shipping them required publishing.
- IGES 5.3 reading is scaffolded on the [`cadmpeg`](https://github.com/cadmpeg/cadmpeg) codecs behind an `iges` feature. The conversion to B-rep is unwritten and returns a typed `Unimplemented` rather than an empty model -- an importer that silently returns nothing is worse than one that refuses.
- A second STEP reader sits behind a `cadmpeg` feature alongside ours. Ours stays the default and the measurement baseline; keeping both compiled lets the two be run over one corpus and diffed, so any future swap is a decision with evidence behind it.

**Geometric Continuity**
- Checked continuity vocabulary in `monstertruck-traits`: `ContinuityOrder` (`G0`--`G4`, with `G4` marked experimental), `BoundarySide` for full tensor-product patch sides, and a capability report carrying a *typed* reason and the highest achievable order rather than a bare boolean.
- `BsplineSurface::continuity_capability` and `NurbsSurface::continuity_capability` inspect knot vectors, clamping, cross-boundary degree, control rows and the parameter domain. Contributed by [@KTheMan](https://github.com/KTheMan) ([#13](https://github.com/virtualritz/monstertruck/pull/13), [#19](https://github.com/virtualritz/monstertruck/pull/19)).

**Testing Infrastructure**
- STEP watertightness invariant + boolean-ops-over-STEP-geometry coverage ([issue #91](https://github.com/virtualritz/monstertruck/issues/91)).
- Assembly STEP round-trip, sphere pole-case property test, end-to-end 2D pcurve STEP integration test.
- 59/59 `monstertruck-solid` tests green under `--features step-test`; meta-crate doctest exercises the full cuboid -> revolved cylinder -> `solid::and` -> `Solid::compress` -> `CompleteStepDisplay` path end-to-end.

**Ported Upstream Commits** (attributed in their commit bodies)
- [`524f5f53`](https://github.com/ricosjp/truck/commit/524f5f53) (revolved-line cylinder STEP), [`08d2cbf1`](https://github.com/ricosjp/truck/commit/08d2cbf1) (`ToSameGeometry` for STEP 2D primitives), [`6c135abc`](https://github.com/ricosjp/truck/commit/6c135abc) (ASCII STL `solid ` header), partial [`7b1f4171`](https://github.com/ricosjp/truck/commit/7b1f4171) (parameter division guard + jitter decorrelation), [`77e25635`](https://github.com/ricosjp/truck/commit/77e25635) (`BasisWindow`), [`993e156c`](https://github.com/ricosjp/truck/commit/993e156c) (tangent circular arcs), [`9031e6dd`](https://github.com/ricosjp/truck/commit/9031e6dd) (offset geometry), `213-assy-step-output`/[`0394eb43`](https://github.com/ricosjp/truck/commit/0394eb43)/[`82114a04`](https://github.com/ricosjp/truck/commit/82114a04) (assembly STEP output), [`f563ae53`](https://github.com/ricosjp/truck/commit/f563ae53) + [`86e4ed75`](https://github.com/ricosjp/truck/commit/86e4ed75) (`UnitCircle` hint honoring).
- Upstream PRs we merged back **into truck** before forking: [ricosjp/truck#40](https://github.com/ricosjp/truck/pull/40) (canonical `struct` naming, dep bumps), [ricosjp/truck#48](https://github.com/ricosjp/truck/pull/48) (removed `_get` prefixes; `Mutex`/`Arc` swapped for faster alternatives).

### Keeping in Sync with `truck`

We do **not** merge or rebase `truck`. Because every crate has been renamed and
much of the public API reworked, a merge would collide wholesale and would
reintroduce upstream code we have deliberately diverged from. Instead we treat
upstream as a patch queue: each useful commit is hand-ported with attribution,
audited against our naming and architecture, and verified by our own tests.

Current parity, tracked per crate and feature, lives in
[`TRUCK-PARITY.md`](TRUCK-PARITY.md). The narrative survey and porting
rationale are in [`truck-sync.md`](truck-sync.md).

## Usage

All `monstertruck-*` crates are released in lockstep and share one version
number; internal dependencies are pinned to the same minor, so mixing crate
versions across a release boundary is unsupported. Depend either on the
`monstertruck` facade crate or on the individual `monstertruck-*` crates you
need, at the same version:

```bash
cargo add monstertruck
# or, granular:
cargo add monstertruck-modeling monstertruck-solid monstertruck-meshing
```

### Running the Examples

All examples are located under the `examples` directory within each respective crate. They use standard Cargo syntax for execution.

To test-drive `monstertruck` and render your first object, run the following commands:

```bash
# Clone the required submodules
git submodule update --init

# Run the basic rotation example
cargo run --example rotate-objects
```

## Architecture & Crate Ecosystem

The `monstertruck` kernel is split into independent crates so you only need to pull in what you need (and also to help with build times).

### Core & Geometry

- [`monstertruck-core`](monstertruck-core/) -- Core types and traits for linear algebra, curves, surfaces, and tolerances.
- [`monstertruck-derive`](monstertruck-derive/) -- Derive macros for geometric traits.
- [`monstertruck-traits`](monstertruck-traits/) -- Geometric trait definitions.
- [`monstertruck-geometry`](monstertruck-geometry/) -- Geometric primitives: knot vectors, B-splines, NURBS, and T-splines.

### Topology & Modeling

- [`monstertruck-topology`](monstertruck-topology/) -- Topological data structures: vertices, edges, wires, faces, shells, and solids.
- [`monstertruck-modeling`](monstertruck-modeling/) -- Integrated geometric and topological modeling algorithms.
- [`monstertruck-solid`](monstertruck-solid/) -- Boolean operations for solids.
- [`monstertruck-fillet`](monstertruck-fillet/) -- Rolling-ball fillets and chamfers on shell edges, a post-CSG pass.
- [`monstertruck-healing`](monstertruck-healing/) -- Shape healing for solids imported from other CAD systems, also post-CSG.
- [`monstertruck-assembly`](monstertruck-assembly/) -- Assembly data structures using a directed acyclic graph (DAG).

### Meshing & Rendering

- [`monstertruck-mesh`](monstertruck-mesh/) -- Polygon mesh data structures and algorithms.
- [`monstertruck-meshing`](monstertruck-meshing/) -- Tessellation and meshing algorithms for B-rep shapes.
- [`monstertruck-gpu`](monstertruck-gpu/) -- Graphics utility crate built on `wgpu`.
- [`monstertruck-render`](monstertruck-render/) -- Shape and polygon mesh visualization.

### I/O & Bindings

- [`monstertruck-io`](monstertruck-io/) -- CAD exchange formats, one feature per format: STEP (read and write, on by default), IGES (placeholder).
- [`monstertruck-step`](monstertruck-step/) -- **deprecated**, a re-export of `monstertruck-io`'s `step` module.
- [`monstertruck-wasm`](monstertruck-wasm/) -- Wasm/JavaScript bindings.

## Dependency Graph

![dependencies](./dependencies.svg)

> This graph predates the `monstertruck-fillet` and `monstertruck-healing` extraction and the `monstertruck-io` consolidation, and shows none of them yet.

## License

Apache License 2.0
