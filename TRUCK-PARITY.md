# Upstream parity -- `ricosjp/truck`

Per-crate, feature-by-feature status of `monstertruck` relative to upstream
[`ricosjp/truck`](https://github.com/ricosjp/truck). For the narrative survey,
porting rationale, and the boolean-op regression post-mortem, see
[`truck-sync.md`](truck-sync.md).

## Sync state

- Upstream remote: `https://github.com/ricosjp/truck.git`.
- Merge-base: `9845ce71` (2026-02-12). **Deliberately never advanced** -- we
  hand-port audited commits rather than merge, so the merge-base is only a
  reference point, not a claim of integration.
- Last survey: upstream `efe39930` (2026-08-31), 178 commits ahead of the
  merge-base.
- Surveying without a configured remote: `git fetch --no-tags
  https://github.com/ricosjp/truck.git 'refs/heads/master:refs/truck-survey/master'`
  leaves the objects in place without adding a remote or a tracking branch.
- Policy: treat upstream as a patch queue, not a branch to merge. A merge or
  broad cherry-pick would reintroduce upstream's `truck-shapeops` boolean-op
  rewrite that we reverted (it regressed `punched_cube`/`adjacent_cubes_or`),
  and would collide with the workspace-wide crate/API renames.

Legend: ported = landed in `monstertruck`; deferred = useful but parked as its
own project; skip = not applicable or not worth porting; ahead = `monstertruck`
has diverged beyond upstream.

## Crate name mapping

| Upstream | `monstertruck` |
|---|---|
| `truck-base` | `monstertruck-core` |
| `truck-geotrait` | `monstertruck-traits` |
| `truck-geometry` | `monstertruck-geometry` |
| `truck-topology` | `monstertruck-topology` |
| `truck-polymesh` | `monstertruck-mesh` |
| `truck-meshalgo` | `monstertruck-meshing` |
| `truck-modeling` | `monstertruck-modeling` |
| `truck-shapeops` | `monstertruck-solid` |
| `truck-stepio` | `monstertruck-step` |
| `truck-platform` | `monstertruck-gpu` |
| `truck-rendimpl` | `monstertruck-render` |
| `truck-assembly` | `monstertruck-assembly` |
| `truck-js` | `monstertruck-wasm` |
| `truck-drafting` | `monstertruck-sketch` (only if it ever lands) |

## `monstertruck-core` (`truck-base`)

- ported -- `SurfaceDerivatives` combinatorial/absolute derivatives (`absolute_derivatives`, `combinatorial_derivative(s)`), backing offset geometry.
- skip -- reduce `MAX_DER_ORDER` from `31` to `10` (`fix-max-ders`): no evidence it is justified; kept at `31`.
- skip -- `better-hash` casting generic scalars through `f64` via `ToPrimitive`: conflicts with our precision/generic-number direction.

## `monstertruck-traits` (`truck-geotrait`)

- ported -- surface-division recursion guard (`MAX_PARAMETER_DIVISION_RECURSION`) and independent `hash2` jitter channels (partial `7b1f4171`).
- ahead -- parameter-space markers renamed `D1` -> `CurveParameter`, `D2` -> `SurfaceParameter` (deprecated aliases kept).
- ahead -- scalar-generic `v2` trait family (no upstream equivalent).
- skip -- `fix-geotrait-tests`: test modernization against upstream's trait surface only.

## `monstertruck-geometry` (`truck-geometry`)

- ported -- `BasisWindow` active-window B-spline basis evaluation (`77e25635`), reimplemented with `SmallVec` and our naming; `BsplineCurve`/`BsplineSurface` evaluate only active control points.
- ported -- offset geometry (`9031e6dd`): `OffsetCurve`, `OffsetSurface`, `NormalOffsetField`, `CurveScalarFunction`, `SurfaceScalarFunction` (renamed from upstream's `Offset`/`NormalField`/`ScalarFunctionD*`).
- ported -- `UnitCircle::search_{nearest_}parameter` now honors the `hint` across periods (`f563ae53` + `86e4ed75` clippy), including the `v2` delegations.
- ported -- least-squares fitting for `BsplineCurve` and `BsplineSurface` (`cd368b72` + `8614ce76` + `cc396e38` + `a696134f`): `BsplineCurve::least_square`, `BsplineSurface::least_square`, at upstream's post-`a696134f` shape with our `KnotVector`/`start_index()`/`values()` naming. Both build the banded normal matrix directly from each sample's `BasisWindow` rather than forming the full design matrix.
- ported -- the shared `gaussian_elimination` helper's zero-multiplier guards and the back-substitution inner-loop bound (`0..size + 1` -> `i..size + 1`), the remainder of `a696134f`. **Not an accuracy fix.** Graded against an exact rational solve of the same float matrix, the median error ratio is 1.000 across banded, dense-Bernstein and normal-matrix families, with the few differences running in both directions. It does move the refuse-versus-answer boundary, always safely -- the new code leaves the diagonals exactly as `echelon` wrote them, so it can turn a wrong answer into a refusal but not the reverse. No input reachable through `try_interpolate` was found to distinguish the two variants. Taken for parity and because it does strictly less arithmetic.
- ahead -- `rbf_surface` -> `rolling_ball_fillet`, `af_surface` -> `approximate_fillet_surface` (structure and names diverged; do not resurrect `rbf_surface`).
- ahead -- `KnotVec` -> `KnotVector`, `BSpline*` -> `Bspline*` spelling.
- ported -- sphere coordinate-singularity guards (pole `0/0`, `point == center`, `acos` clamp) -- `monstertruck`-only hardening, no upstream equivalent.

## `monstertruck-topology` (`truck-topology`)

- already-landed -- `BoundingBox::is_empty` checks every dimension (`fix-empty-bounding-box`).
- already-landed -- non-intersecting bounding boxes report no intersection (`0cae5bc5`).
- ahead -- `CompressedShell`/`CompressedSolid` stable-ID plumbing and `Face::try_new` returning `Result`.

## `monstertruck-mesh` (`truck-polymesh`)

- ported -- ASCII STL writes `solid ` with a trailing space before the optional name (`6c135abc`).
- already-landed -- binary STL uses `read_exact` (`stl-binary-read_exact`, `f43020bf`).

## `monstertruck-meshing` (`truck-meshalgo`)

- ahead -- triangulation/tessellation heavily rewritten (CDT trim-constraint handling, conic meshing fixes); diverged from upstream.
- ported -- offset-surface tessellation fix folded into our tessellation path (`7b1f4171` meshing portion).

## `monstertruck-modeling` (`truck-modeling`)

- ported -- tangent-constraint circular arcs (`993e156c`): `CircularArcConstraint::{ThroughPoint, StartTangent}`, `try_circle_arc_by_start_tangent` (renamed from upstream `ArcConstraint::{Transit, Tangent}`/`circle_arc_by_tangent0`).

## `monstertruck-solid` (`truck-shapeops`)

- ahead -- robust boolean ops return `Result<Solid, ShapeOpsError>` (`and`/`or`/`difference`/`symmetric_difference`).
- reverted -- upstream's `700138cb`-equivalent boolean rewrite (multi-ray voting, healing capper, greedy assignment search): regressed `punched_cube`/`adjacent_cubes_or`; we run the upstream-derived single-ray algorithm wrapped in our `Result` layer. See [`truck-sync.md`](truck-sync.md).
- ahead -- `strip_seam_edges` healing pass splits STEP seam wires (one edge twice, opposite orientations) into simple wires; no upstream equivalent.
- reference-only -- fillet branches (`simple-fillet-with-side`, `fix-fillet-estimation`): mine tests/numerical fixes only; do not resurrect old fillet architecture.

## `monstertruck-step` (`truck-stepio`)

- ahead -- `src/in` -> `src/load`, `src/out` -> `src/save`; `LoadError` thiserror enum; `Table::from_step` returns `Result`.
- ported -- revolved-line-to-cylinder surface conversion fix (`524f5f53`), adapted to `RevolutionSurface` naming.
- ported -- `ToSameGeometry` for STEP 2D geometries (`08d2cbf1`): `Line<Point2>`, `Processor<TrimmedCurve<UnitCircle<Point2>>, Matrix3>`, `BsplineCurve<Point2>`.
- ported -- assembly STEP output as `save/assembly.rs` (`213-assy-step-output`/`0394eb43`/`82114a04`): `StepDesign`, `MatrixAsAxis`, renamed `PartAttrs` -> `PartAttributes`, `DisplayByStep` -> `StepFormat`.
- ahead -- `preview-step-face` diagnostic example for visual debugging of meshing/trim bugs (see [AGENTS.md](AGENTS.md)).

## `monstertruck-assembly` (`truck-assembly`)

- ported -- `Default` impls for `NodeEntity`/`EdgeEntity`, `Dag::map`/`par_map` lifetimes -- foundation for assembly STEP output.
- ahead -- `Node`/`Edge` `attrs()` -> `attributes()` (deprecated aliases kept).

## `monstertruck-gpu` (`truck-platform`)/`monstertruck-render` (`truck-rendimpl`)

- ahead -- `truck-platform` renamed to `monstertruck-gpu`; edition 2024, `wgpu` 28.
- skip -- `remove-render-object-by-id` (`79d2bc60`/`329af874`): GPU/render API churn, not kernel correctness.
- skip -- `fix-example-pages-on-mac` (`31dc0e7c`): example/render maintenance.

## `monstertruck-wasm` (`truck-js`)

- ahead -- renamed; consumes the renamed kernel crates.

## `monstertruck-sketch` (`truck-drafting`) -- not integrated

- deferred -- entire crate. Useful 2D-construction reference, not public-API quality yet (panicking wrappers, ambiguous arc-constraint names, scale-dependent arc-length integration, broad prelude re-exports). Upstream keeps expanding it (`line_line`, `arc_arc`, `line_arc_line` in `6b2e59b4`; multi-edge connectors).
- mine-only -- tangent circular arc construction (already pulled into `monstertruck-modeling`); fillet/chamfer tests as future acceptance tests once the crate's API is cleaned up.
- See [`truck-sync.md`](truck-sync.md) for the full naming/robustness cleanup checklist required before integration.

## Routinely skipped upstream commit classes

- `cargo upgrade` dependency rolls.
- `Update CHANGELOG`/changelog-only commits.
- `fmt`/`clippy`/`dos2unix` cosmetic commits (unless they touch code we are porting).
- Merge commits.
