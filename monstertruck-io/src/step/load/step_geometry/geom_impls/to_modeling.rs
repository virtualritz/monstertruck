//! Handing STEP geometry over to `monstertruck-modeling`: which STEP
//! carriers reach an analytic modeling variant, and which fall through to
//! the rational net.

use super::*;

fn to_modeling_trim(
    curve: &StepParameterCurve,
) -> std::result::Result<ParameterCurve<ModelingCurve2D, Box<ModelingSurface>>, StepConvertingError>
{
    Ok(ParameterCurve::new(
        curve.curve().as_ref().try_into()?,
        Box::new(curve.surface().as_ref().try_into()?),
    ))
}

impl TryFrom<&Curve3D> for ModelingCurve {
    type Error = StepConvertingError;
    fn try_from(value: &Curve3D) -> std::result::Result<Self, Self::Error> {
        match value {
            Curve3D::Line(line) => Ok((*line).into()),
            Curve3D::BsplineCurve(curve) => Ok(curve.clone().into()),
            Curve3D::NurbsCurve(curve) => Ok(curve.clone().into()),
            Curve3D::ParameterCurve(curve) => {
                Ok(ModelingCurve::ParameterCurve(ParameterCurve::new(
                    curve.curve().as_ref().try_into()?,
                    Box::new(curve.surface().as_ref().try_into()?),
                )))
            }
            Curve3D::SurfaceCurve(curve) => {
                let surfaces = curve
                    .associated_geometry()
                    .iter()
                    .map(SurfaceCurveAssociatedGeometry::surface)
                    .collect::<Vec<_>>();
                if surfaces.len() >= 2 {
                    let surface0 = surfaces[0].try_into()?;
                    let surface1 = surfaces[1].try_into()?;
                    let boundary0 = curve
                        .parameter_curve_on(surfaces[0])
                        .cloned()
                        .map(|trim| to_modeling_trim(&trim))
                        .transpose()?;
                    let boundary1 = curve
                        .parameter_curve_on(surfaces[1])
                        .cloned()
                        .map(|trim| to_modeling_trim(&trim))
                        .transpose()?;
                    Ok(ModelingCurve::IntersectionCurve(
                        SurfaceCurve::with_boundaries(
                            Box::new(surface0),
                            Box::new(surface1),
                            Box::new(curve.leader().try_into()?),
                            boundary0,
                            boundary1,
                        ),
                    ))
                } else {
                    curve.leader().try_into()
                }
            }
            Curve3D::IntersectionCurve(curve) => Ok(ModelingCurve::IntersectionCurve(
                SurfaceCurve::with_boundaries(
                    Box::new(curve.surface0().as_ref().try_into()?),
                    Box::new(curve.surface1().as_ref().try_into()?),
                    Box::new(curve.leader().as_ref().try_into()?),
                    None,
                    None,
                ),
            )),
            _ => value
                .try_into_homogeneous_bspline_curve()
                .map(|curve| ModelingCurve::NurbsCurve(NurbsCurve::new(curve)))
                .ok_or_else(|| "STEP curve cannot be represented in modeling geometry.".into()),
        }
    }
}

impl TryFrom<&Conic2D> for ModelingConic2D {
    type Error = StepConvertingError;
    fn try_from(value: &Conic2D) -> std::result::Result<Self, Self::Error> {
        match value {
            Conic2D::Ellipse(curve) => Ok((*curve).into()),
            Conic2D::Hyperbola(curve) => Ok((*curve).into()),
            Conic2D::Parabola(curve) => Ok((*curve).into()),
        }
    }
}

impl TryFrom<&Curve2D> for ModelingCurve2D {
    type Error = StepConvertingError;
    fn try_from(value: &Curve2D) -> std::result::Result<Self, Self::Error> {
        match value {
            Curve2D::Line(curve) => Ok((*curve).into()),
            Curve2D::Polyline(curve) => Ok(curve.clone().into()),
            Curve2D::Conic(curve) => Ok(ModelingCurve2D::Conic(curve.try_into()?)),
            Curve2D::BsplineCurve(curve) => Ok(curve.clone().into()),
            Curve2D::NurbsCurve(curve) => Ok(curve.clone().into()),
        }
    }
}

/// Whether the analytic sphere route is admissible, restating the guard in
/// `TryIntoHomogeneousBsplineSurface for Sphere` (`bspline_conversion.rs`).
///
/// The predicate has to be the BUILDER's and no wider: a sphere that fails it
/// must reach the generic arm, where the builder returns `None` and the
/// conversion refuses -- which is what it does today.
/// Whether STEP spheres route onto the analytic [`ModelingSurface::SphericalSurface`].
///
/// **`false` -- STILL HELD, but NOT for the reason the previous revision gave.**
/// Ledger class C14.
///
/// # The previous diagnosis, and its falsification
///
/// This arm was reverted on 2026-07-31 with the finding "it changed the
/// GEOMETRY, not merely its tessellation cost", bisected on
/// `rotor_sphere_pin_a_union_refuses_ambiguous_topology_sign`, which reads ROTOR
/// #19264's extracted mesh volume against a pinned `551.5116` and sees
/// `338.6367` once the arm is on.
///
/// **That reading was FALSIFIED by measurement on 2026-07-31.** The geometry did
/// change -- and it changed toward being CORRECT. `551.5116` is not the solid's
/// volume and is not a converged quantity.
///
/// # What the solids actually are, and their closed forms
///
/// Both sphere pins are the same shape, read off their own loaded boundary
/// geometry rather than assumed: a sphere of radius `R = 12.5` centred at the
/// origin, cut by planes at `x = +-h`, with a bore of radius `r` along the
/// x-axis (#19264: `h = 8`, `r = 7.5`; #25387: `h = 9`, `r = 6`). Two
/// half-sphere faces split at `z = 0`, two half-bore faces, two end annuli.
/// The trim circles land at `sqrt(R^2 - h^2)` = `9.604686` / `8.674676`, which
/// is what the loaded edges report to 15 digits.
///
/// By Archimedes' hat-box theorem ONE half-sphere face's contribution to the
/// divergence-theorem x-flux is exactly `pi * 2 h^3 / 3` -- `1072.3303` and
/// `1526.8140` -- and its area is exactly `2 pi R h` = `628.3185` / `706.8583`.
/// Neither depends on the mesh. Measured per face:
///
/// | route | face | x-flux | vs closed form | area |
/// |---|---|---|---|---|
/// | net (`false`) | #19264 z<0 | 1075.6349 | +0.31% | 627.7081 |
/// | net (`false`) | #19264 z>0 | 1272.1296 | **+18.63%** | **265.9786** |
/// | analytic (`true`) | #19264 both | 1067.4449 | -0.46% | 627.4176 |
/// | net (`false`) | #25387 z<0 | 1528.8421 | +0.13% | -- |
/// | net (`false`) | #25387 z>0 | 2000.9617 | **+31.06%** | -- |
/// | analytic (`true`) | #25387 both | 1519.4105 | -0.49% | 705.6795 |
///
/// Refining the ANALYTIC arm walks each face monotonically up to its closed form
/// from below, as an inscribed mesh must (#19264: `1067.4449 -> 1069.6015 ->
/// 1070.5880` against `1072.3303`; areas `627.4176 -> 627.9058 -> 628.0886`
/// against `628.3185`). Refining the NET arm does not converge at all: its bad
/// face goes `1272.1296 -> 1018.5026`, from 18.6% above the closed form to 5.0%
/// below it, and the whole-shell sum goes `551.5116 -> 281.4251`.
///
/// # The mechanism: TRIM INTERPRETATION, on the rational net
///
/// Under the net route one of the two half-sphere faces -- the one straddling
/// the net's periodic seam -- is triangulated over the wrong sub-region of
/// parameter space and covers **42% of its own area** (`265.98` of `628.32`),
/// which refinement lifts only to 45%. Face ORIENTATION is not the mechanism
/// (both routes give the same sign on both faces), and nothing is dropped or
/// duplicated (`face_drop_count() == 0`, six faces, closed shell, on both
/// routes). The analytic route does not have the defect because it never leaves
/// the closed form.
///
/// # The arm is no longer held -- it was re-landed at `d212e597`
///
/// It had been held only because switching it on moved `551.5116` and
/// `1323.4471`, `CorpusSolid::volume` IDENTITY pins in `corpus_boolean_rows.rs`,
/// and re-pinning them was an owner decision rather than a code fix. That
/// decision was taken; the constant below is `true`. This section is kept
/// because the reasoning is what the re-landing rested on.
///
/// # Two further, INDEPENDENT defects this work uncovered (not C14)
///
/// The instrument both sides of C14 were argued with -- a divergence-theorem sum
/// over the whole shell -- was not a volume on these solids. `occt-sphere.step`
/// measured `-523.58` on a `+523.5988` ball and `occt-cube.step` `-1000` on a
/// 1000-volume cube, while `primitive::cuboid` measured `+24` exactly on a
/// 2x3x4 box and `occt-cylinder` / `occt-cone` / `occt-torus` were all correct
/// and positive. Bit-identical with this switch on and off, so it predated and
/// survived the routing question.
///
/// **Spec 013 V1 found TWO mechanisms behind that, not one, and the assumption
/// that the cube and the sphere shared a defect was wrong.**
///
/// 1. **C15 proper, the cube and ROTOR #19264's annuli/bore faces.** A STEP
///    `ADVANCED_FACE`'s loops are oriented about the FACE normal, but
///    `CompressedFace` stores boundaries in the SURFACE sense; the loader passed
///    them through, so every `same_sense = .F.` face was traversed backwards and
///    the shell loaded `Regular`. Fixed in `Table::absolute_bound_orientation`
///    (`load/convert.rs`), with the symmetric `FACE_BOUND` flag on the save side.
/// 2. **A meshing defect, the sphere.** `occt-sphere` is a ONE-face shell, so it
///    cannot have an inconsistent orientation and never did. Its winding was
///    wrong because `ensure_winding_matches_normals` normalizes each face
///    normal, a sphere's pole strip is degenerate, and one `0/0 = NaN` term made
///    `vote < 0.0` false. Fixed in `monstertruck-meshing`.
///
/// The oracle for both is `occt_sphere_extracts_to_the_analytic_ball` (now the
/// SIGNED closed form) plus `monstertruck-healing/tests/step_shell_orientation.rs`.
///
/// # The oracle, and the proof it discriminates
///
/// `rotor_sphere_faces_carry_their_closed_form_x_flux` (`corpus_boolean_rows.rs`)
/// asserts each half-sphere face against `pi * 2 h^3 / 3` in a 1% band.
/// Measured both ways: it FAILS on the net route (+18.632% / +31.055%) and
/// PASSES on the analytic route (-0.456% / -0.485%). While this arm is held the
/// row additionally admits the ONE named net-route value per solid, so the tree
/// stays green; `MT_C14_FORCE_ANALYTIC_BAND=1` drops that escape and reproduces
/// the failure on demand.
///
/// The TORUS sibling below stays ON for the same kind of reason it always did:
/// it carries an analytic oracle
/// (`occt_torus_intersection_with_an_enclosing_box_is_the_torus`, volume
/// 789.5072 against the closed form `2*PI^2*R*r^2 = 789.5684`) and passes it.
///
/// TO RE-LAND: re-pin `ROTOR_SPHERE_PIN_A::volume` and `ROTOR_SPHERE_PIN_B::volume`
/// to the analytic arm's measurements, flip this constant, and confirm the
/// closed-form row above. The display win is real (ROTOR #19264 UNBOUNDED at
/// chord 1e-3 -> 5.1 s).
///
/// Public so a test in another crate can assert WHICH surface a loaded sphere
/// face reached the kernel as, in BOTH states of this switch -- an oracle that
/// does not know which route it measured cannot certify either one.
pub const ROUTE_ANALYTIC_SPHERE: bool = true;

fn analytic_sphere_is_representable(sphere: &monstertruck_geometry::prelude::Sphere) -> bool {
    let radius = sphere.radius();
    radius.is_finite() && radius > TOLERANCE
}

/// Whether the analytic torus route is admissible.
///
/// Restates `TryIntoHomogeneousBsplineSurface for Torus`
/// (`bspline_conversion.rs`) verbatim, INCLUDING its spindle rejection. Spec
/// 011 T1: on a spindle torus (`small - large` above the relative tolerance)
/// the surface passes through itself and `search_parameter` is silently wrong
/// on roughly a third of the domain, so the class refuses typed. Horn tori
/// (`large == small`, including the near-horn fillets real STEP files carry a
/// few ulps below) ARE representable and must keep converting.
fn analytic_torus_is_representable(torus: &Torus) -> bool {
    let (large_radius, small_radius) = (torus.large_radius(), torus.small_radius());
    large_radius.is_finite()
        && small_radius.is_finite()
        && small_radius > TOLERANCE
        && large_radius > TOLERANCE
        && small_radius - large_radius <= TOLERANCE * (large_radius + small_radius)
}

/// STEP surface -> modeling surface.
///
/// Cylinders and cones map onto the ANALYTIC
/// [`ModelingSurface::RevolutionSurface`] variant rather than being flattened.
/// The flattening path is lossy in a way nothing downstream can undo: a STEP
/// `CYLINDRICAL_SURFACE` is a revolution of a UNIT-LENGTH profile line, so the
/// untrimmed homogeneous conversion emits a control net spanning ONE AXIAL UNIT
/// of an unbounded surface, and no consumer can widen a 4x2 rational net back
/// out to the extent the face actually occupies. The kernel then reports a
/// CONFIDENT empty for face pairs that demonstrably intersect -- 92 of boxy's
/// 126 pairs before this mapping (spec 010, T22).
///
/// Measured on the boxy union: `OK curves=0` falls 100 -> 59 over the 126-pair
/// census with **no pair regressing** (all 26 already-tracing pairs
/// byte-identical), and six pairs move from a silent `Ok(vec![])` to an honest
/// `SsiFailed`. The alternative of keeping the NURBS representation and
/// re-spanning it over each face's trims was measured and REJECTED: it emits
/// one surface carrying two parameter conventions -- the angular axis
/// renormalized to `[0, 1]` while the axial axis keeps model-space knots -- so
/// the angular origin is unrecoverable, and it regressed six pairs from two
/// traced curves to zero. See `FIX_PLAN_010_PRODUCER_TRACK.md` sections 7m/7n.
///
/// Flipping to the analytic variant moves `supports_exact_patch_domains` to
/// `false` and `parameter_range` to `((0, 1), (0, 2pi))` for these surfaces,
/// which is what lets the broad phase see their true extent. The save side
/// already round-trips this variant back to a STEP `CYLINDRICAL_SURFACE`
/// (`save/geometry.rs`), so it improves save fidelity rather than costing it.
///
/// # Spheres and tori (spec 012 U1.2), same shape, different axis
///
/// T22 above was about the emitted net's EXTENT. Spheres and tori never had
/// that defect -- their dedicated rational builders are machine-exact over the
/// whole domain (ledger C1, "not this class", 7y) -- but routing them through
/// [`TryIntoHomogeneousBsplineSurface`] threw away something else the analytic
/// form carries: their CLOSED-FORM [`ParameterDivision2D`]
/// (`specifieds/sphere.rs`, `specifieds/torus.rs`). The generic net divider
/// then has to discover a sphere's curvature by adaptive bisection.
///
/// Measured over ROTOR's five T4 solids, 169 faces, at the guard's `1e-3`:
/// **35,053 refinement cells on the STEP side against 8,469,082 on the modeling
/// side, and 8,434,029 of that gap -- 99.6% -- is these two classes**, which
/// spend ZERO on the STEP side. A six-face solid (#19264: two spheres, two
/// cylinders, two planes) took 116.3 s to mesh for DISPLAY.
///
/// So they map onto analytic variants too. What that costs, enumerated:
/// `try_into_homogeneous_bspline_surface` on the new variants is the SAME call
/// on the SAME `Processor<_, Matrix4>` this arm used to make eagerly, so the
/// net the boolean prepares is byte-identical and only its construction moved
/// from load time to use time. `supports_exact_patch_domains` flips `true` ->
/// `false`, exactly as T22's flip did. `search_parameter` becomes the analytic
/// inverse instead of a Newton descent on a net. Nothing in the boolean engine,
/// the topology crate or the mesher matches on `Surface`'s variants, so the
/// dispatch cost is confined to `monstertruck-modeling`'s own `geometry.rs`,
/// this crate's save side, and `fillet_impl.rs`.
///
/// # The degenerate torus stays refused (spec 011 T1)
///
/// The refusal for `|large| < small` lives in the BUILDER
/// (`bspline_conversion.rs`), so a routing change that stops calling the
/// builder would silently reopen it -- and it must not: on a spindle the
/// FORWARD map is exact to 8e-16 while `search_parameter` is wrong on ~29% of
/// the domain, which is what places trims. [`analytic_torus_is_representable`]
/// therefore restates the builder's predicate verbatim, and a torus that fails
/// it falls through to the generic arm, where the builder returns `None` and
/// the conversion refuses exactly as it does today. Pinned by
/// `spindle_torus_parameter_recovery_is_unsound_while_ring_and_horn_are_exact`
/// (`monstertruck-geometry/tests/torus.rs`) and by
/// `a_spindle_torus_is_still_refused_by_the_analytic_route` below.
impl TryFrom<&Surface> for ModelingSurface {
    type Error = StepConvertingError;
    fn try_from(value: &Surface) -> std::result::Result<Self, Self::Error> {
        match value {
            Surface::ElementarySurface(ElementarySurface::Plane(surface)) => Ok((*surface).into()),
            // Both arrive as `Processor<RevolutionSurface<Line<Point3>>, Matrix4>`,
            // so one or-pattern binds them. `map_ref` carries the `Matrix4` and
            // the processor's orientation across untouched; only the profile is
            // lifted into the modeling curve enum.
            Surface::ElementarySurface(
                ElementarySurface::CylindricalSurface(surface)
                | ElementarySurface::ConicalSurface(surface),
            ) => Ok(ModelingSurface::RevolutionSurface(surface.map_ref(
                |revolution| {
                    RevolutionSurface::by_revolution(
                        ModelingCurve::Line(*revolution.entity_curve()),
                        revolution.origin(),
                        revolution.axis(),
                    )
                },
            ))),
            // `map_ref` again: the `Matrix4` and the orientation flag ride
            // across untouched, only the STEP newtype's `(u, v)` relabeling is
            // dropped. That relabeling has Jacobian determinant +1, so the
            // composite orientation -- and therefore the surface normal -- is
            // unchanged either way. Face trims cannot be affected: they are
            // ERASED (`TrimmedSolid::erase_trims`) before the geometry is
            // mapped, and every consumer re-derives `(u, v)` by projecting the
            // 3D boundary onto the modeling surface.
            Surface::ElementarySurface(ElementarySurface::Sphere(surface))
                if ROUTE_ANALYTIC_SPHERE
                    && analytic_sphere_is_representable(&surface.entity().0) =>
            {
                Ok(ModelingSurface::SphericalSurface(
                    surface.map_ref(|sphere| sphere.0),
                ))
            }
            // TORUS ROUTING, spec 012 W1: the sibling of the sphere arm above,
            // and it was HELD BACK behind an `if false &&` for one round.
            //
            // The stated reason to hold was that switching it on MOVED ap224's
            // pinned refusal from `UnknownClassificationFailed` to
            // `CreateLoopsStoreFailed{IntersectionCurvesFailed{(15,4),
            // SsiFailed}}`, read as "SSI cannot intersect the analytic torus
            // where it could intersect the NURBS form". **That reading was
            // FALSIFIED by measurement.** With
            // `MT_SSI_DEBUG_EXCLUSIONS`/`MT_SSI_DEBUG_TRIM_FILTER` on the
            // failing pair, the SSI backend tested 16 patch pairs, passed 2,
            // and traced 2 core curves -- the RIGHT ones. The SSI was fine.
            // What failed was the FACE: `trim_rejected=2`, `side0=0` on all 8
            // segments, i.e. every traced point tested outside face 15's own
            // parameter loop, because `Torus::search_parameter` discarded the
            // caller's hint and spelled the seam vertex a whole period away
            // from its neighbours. Ledger class C4, fixed at the source
            // (`monstertruck-geometry/src/specifieds/torus.rs`,
            // `nearest_periodic_angle`), and with it:
            //
            //   * ap224 face 15's `u` trim range: `(0.0645, 6.2832)` -- the
            //     whole ring -- becomes `(0, PI)`, and face 19's becomes
            //     `(PI, 2 PI)`. Both now agree with what the same faces report
            //     over the rational net, to the padding.
            //   * ZERO SSI face-pair errors on the ap224 union, and
            //     `ap224_main_solid_union_refuses_typed` passes on its
            //     ORIGINAL pin, `UnknownClassificationFailed{shell_index: 1}`.
            //     Nothing had to be re-pinned.
            //
            // Coverage, the other reason it was held: `occt_torus_*` in
            // `user_fixture_boolean_tests` now carries a torus-bearing boolean
            // that SUCCEEDS against a closed-form volume, so the capability is
            // covered rather than only its refusal.
            //
            // The guard below is the T1 spindle predicate restated verbatim, so
            // spec 011's degenerate-torus refusal survives this unchanged.
            Surface::ElementarySurface(ElementarySurface::ToroidalSurface(surface))
                if analytic_torus_is_representable(surface.entity()) =>
            {
                // `*`, not `.clone()`: `Processor<Torus, Matrix4>` is `Copy`.
                // The dead arm carried the clone unnoticed -- clippy does not
                // lint through an `if false` guard, which is one more reason a
                // held-back arm is not a free thing to leave lying around.
                Ok(ModelingSurface::ToroidalSurface(*surface))
            }
            _ => value
                .try_into_homogeneous_bspline_surface()
                .map(|surface| ModelingSurface::NurbsSurface(NurbsSurface::new(surface)))
                .or_else(|| {
                    value
                        .try_into_bspline_surface()
                        .map(ModelingSurface::BsplineSurface)
                })
                .ok_or_else(|| "STEP surface cannot be represented in modeling geometry.".into()),
        }
    }
}

// ---------------------------------------------------------------------------
// Spec 012 U1.2: the analytic sphere/torus route.
// ---------------------------------------------------------------------------

#[cfg(test)]
fn u1_step_sphere(radius: f64) -> Surface {
    Surface::ElementarySurface(ElementarySurface::Sphere(Processor::new(Sphere(
        monstertruck_geometry::prelude::Sphere::new(Point3::new(1.0, -2.0, 3.0), radius),
    ))))
}

#[cfg(test)]
fn u1_step_torus(large_radius: f64, small_radius: f64) -> Surface {
    Surface::ElementarySurface(ElementarySurface::ToroidalSurface(Processor::new(
        Torus::new(Point3::new(1.0, -2.0, 3.0), large_radius, small_radius),
    )))
}

/// The routing change is a TESSELLATION change and nothing else: the rational
/// net the boolean's homogeneous path prepares must be the very same one it
/// prepared while the modeling surface WAS that net.
///
/// Byte-identity, not a tolerance -- both sides call the same builder on the
/// same `Processor<_, Matrix4>`, so anything short of equality would mean the
/// route picked up an extra arithmetic step (the C1 re-spanning trap).
#[test]
fn the_analytic_route_emits_the_same_rational_net_the_flattened_route_did() {
    for step in [u1_step_sphere(53.0), u1_step_torus(7.0, 2.0)] {
        let flattened = step
            .try_into_homogeneous_bspline_surface()
            .expect("the builder accepts these radii");
        let modeling = ModelingSurface::try_from(&step).expect("must convert");
        assert!(
            matches!(
                modeling,
                ModelingSurface::SphericalSurface(_) | ModelingSurface::ToroidalSurface(_)
            ),
            "spheres and tori must reach the analytic variants, got {modeling:?}",
        );
        let analytic = modeling
            .try_into_homogeneous_bspline_surface()
            .expect("the analytic variant must still yield its net");
        assert_eq!(
            flattened.knot_vectors(),
            analytic.knot_vectors(),
            "the emitted knot vectors moved",
        );
        assert_eq!(
            flattened.control_points(),
            analytic.control_points(),
            "the emitted control net moved",
        );
    }
}

/// The closed form is what the whole track is for: the analytic variants must
/// spend ZERO adaptive refinement cells where the net spent them by the
/// million.
///
/// A count, not a wall clock (ledger M13/C8): `division_totals` is the
/// process-wide cell counter the U1.1 budget already maintains.
#[test]
fn the_analytic_route_spends_no_adaptive_refinement_cells() {
    use monstertruck_traits::algo::surface::take_division_totals;
    for (step, range) in [
        (
            u1_step_sphere(53.0),
            ((0.0, std::f64::consts::PI), (0.0, TAU)),
        ),
        (u1_step_torus(7.0, 2.0), ((0.0, TAU), (0.0, TAU))),
    ] {
        let modeling = ModelingSurface::try_from(&step).expect("must convert");
        let net = monstertruck_geometry::prelude::NurbsSurface::new(
            step.try_into_homogeneous_bspline_surface().unwrap(),
        );
        // Each side over ITS OWN declared range -- the analytic one in radians,
        // the net over its knot span. Comparing them over one frame would be
        // the C2 trap, and would also stack the extrapolation defect (b) onto
        // a measurement that is about the closed form (a).
        let net_range = net.range_tuple();

        let _ = take_division_totals();
        let _ = modeling.parameter_division(range, 1.0e-3);
        let (analytic_cells, _) = take_division_totals();

        let _ = net.parameter_division(net_range, 1.0e-3);
        let (net_cells, _) = take_division_totals();

        assert_eq!(
            analytic_cells, 0,
            "the analytic variant must divide in closed form",
        );
        assert!(
            net_cells > 0,
            "the net must still cost what it always cost, else this test proves nothing",
        );
    }
}

/// Spec 011 T1 must survive the routing change.
///
/// The spindle refusal lives in the BUILDER, so an analytic route that stopped
/// calling the builder would reopen it silently. On a spindle the FORWARD map
/// stays exact while `search_parameter` is wrong on ~29% of the domain, which
/// is what places trims -- so the class must keep refusing typed in every
/// encoding, and horn tori (the fillet form) must keep converting.
#[test]
fn a_spindle_torus_is_still_refused_by_the_analytic_route() {
    // |large| < small in each of the spellings the corpus carries.
    for (large_radius, small_radius) in [(1.0, 3.0), (0.5, 20.0), (1.0e-3, 1.0)] {
        let step = u1_step_torus(large_radius, small_radius);
        assert!(
            ModelingSurface::try_from(&step).is_err(),
            "spindle torus (R = {large_radius}, r = {small_radius}) must refuse typed",
        );
    }
    // Horn (R == r) and ring (R > r) are unaffected and must still convert.
    for (large_radius, small_radius) in [(2.0, 2.0), (7.0, 2.0)] {
        let step = u1_step_torus(large_radius, small_radius);
        assert!(
            matches!(
                ModelingSurface::try_from(&step),
                Ok(ModelingSurface::ToroidalSurface(_))
            ),
            "torus (R = {large_radius}, r = {small_radius}) must convert typed",
        );
    }
}

/// The guard is the BUILDER's and no wider: a surface the analytic route
/// declines must land on exactly the answer it lands on today, not on a
/// different refusal and not on a silent success.
#[test]
fn the_analytic_guard_matches_the_builders_own_predicate() {
    for step in [
        u1_step_sphere(f64::INFINITY),
        u1_step_torus(1.0, 3.0),
        u1_step_torus(f64::INFINITY, 1.0),
    ] {
        assert_eq!(
            step.try_into_homogeneous_bspline_surface().is_none(),
            ModelingSurface::try_from(&step).is_err(),
            "the routing guard and the builder must agree on {step:?}",
        );
    }
}
