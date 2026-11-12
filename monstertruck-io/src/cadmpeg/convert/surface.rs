//! Surface carriers: intermediate representation to [`Surface`].
//!
//! # Analytic carriers stay analytic
//!
//! Which variant an analytic surface lands on is a decision with consequences,
//! not a formality. monstertruck keeps spheres and tori as their analytic types
//! rather than as their (exact) rational nets so that the CLOSED-FORM parameter
//! division and the analytic `search_parameter` survive the import -- spec 012
//! U1.2. Routing them onto a net instead is exact in the forward direction and
//! wrong in the inverse one, which is the direction that places face trims.
//!
//! The routing here is the one
//! `monstertruck-io/src/step/load/step_geometry/geom_impls/to_modeling.rs` makes
//! for STEP, restated for a different input vocabulary. It is NOT a second
//! opinion about which surfaces are exactly representable.
//!
//! # How the degenerate cases are refused
//!
//! The STEP path guards its analytic arms with predicates that restate the
//! builder's own admissibility rules -- a spindle torus must not reach the
//! analytic variant, because on one `search_parameter` is wrong over ~29% of the
//! domain. Restating a predicate is exactly the thing that drifts, and this
//! module cannot even share the STEP one: `iges` does not imply the `step`
//! feature.
//!
//! So this module does not restate it. It builds the analytic carrier, then ASKS
//! THE BUILDER by probing [`TryIntoHomogeneousBsplineSurface`]. A carrier the
//! builder refuses to net is a carrier whose analytic form is not admissible, and
//! the probe is the predicate rather than a copy of it. One throwaway net per
//! analytic surface is the price, and it cannot drift.
//!
//! # Which way the converted surface faces
//!
//! A face's sense in the intermediate representation is relative to its
//! carrier's ORIENTED normal, and since cadmpeg 0.6 that is not always the
//! natural normal `Su x Sv` of the carrier's own chart. Two carriers state it
//! differently, and the compiler sees neither:
//!
//! * a NURBS net carries `normal_reversed`: when set, the oriented normal is
//!   `-(Pu x Pv)`. The Parasolid-backed codecs set it.
//! * a sphere's radius is SIGNED. A negative radius evaluates the same point
//!   set through the antipodal chart, whose `Su x Sv` points INTO the sphere.
//!   A torus's tube radius is signed too, but there the sign shifts the tube
//!   angle by half a turn and leaves `Su x Sv` pointing out of the tube, so it
//!   does not flip. Both facts are cadmpeg's own: they are what its STEP writer
//!   records as `WrittenChart::reverses_normal` when it writes such a carrier
//!   with a positive radius.
//!
//! So [`convert`] returns the surface together with whether its natural normal
//! OPPOSES the carrier's oriented normal, and the topology layer folds that into
//! the face's orientation exactly once. Pre-inverting the surface instead would
//! work for a plane and silently break the analytic sphere, whose `Processor`
//! inversion swaps its parameters.

use cadmpeg_ir::geometry::nurbs::NurbsSurface as IrNurbsSurface;
use cadmpeg_ir::geometry::{SolvedSurfaceGeometry, SurfaceGeometry};
use monstertruck_core::cgmath64::control_point::ControlPoint;
use monstertruck_geometry::prelude::{
    BsplineSurface, Invertible, KnotVector, NurbsSurface, Processor, RevolutionSurface, Sphere,
    Tolerance, Torus, TryIntoHomogeneousBsplineSurface,
};
use monstertruck_modeling::{
    Curve, Line, Plane, Point3, SquareMatrix, Surface, Transformed, Vector4,
};

use super::{Context, frame};
use crate::Result;

/// A converted carrier and how it faces relative to the source.
#[derive(Debug, Clone)]
pub(super) struct Oriented {
    /// The monstertruck surface.
    pub(super) surface: Surface,
    /// Whether `surface`'s natural normal is OPPOSITE the source carrier's
    /// oriented normal. The face's orientation must absorb this.
    pub(super) reversed: bool,
}

impl Oriented {
    fn forward(surface: Surface) -> Self {
        Self {
            surface,
            reversed: false,
        }
    }
}

/// Convert one surface carrier.
///
/// # Procedural carriers
///
/// Since cadmpeg 0.6 a carrier the source defines by a construction (a
/// revolution, an extrusion, an offset) is `Procedural`, optionally with the
/// solved carrier the source retained as its cache. The representation
/// evaluates such a carrier THROUGH its cache (`SurfaceGeometry::solved`), so
/// the cache is the carrier as far as every reader is concerned, and the
/// face's sense is relative to it.
///
/// A cache is admitted only when the construction states NO cache fit
/// tolerance, which is the representation's way of saying the cache is the
/// construction exactly rather than a fit to it; a NURBS cache that carries its
/// own fit tolerance is refused below in `nurbs`.
///
/// This arm is a POLICY for codecs that emit exact procedural caches, not a
/// measured rescue. Neither shipped decoder reaches it as of cadmpeg 0.6.11:
/// the IGES codec emits an entity 120 revolution as `Solved(Nurbs)` when its
/// generatrix is a NURBS, and as `Procedural { cache: None }` otherwise, and the
/// STEP codec emits `cache: Some` only from its test support. The 20
/// robot-rodney sheets the bridge refuses are the `cache: None` arm, not a
/// cache this code admits, and the 0.5-to-0.6.11 move went from 641 of 666
/// converted bodies to 621 of 641 rather than from 0.
///
/// A procedural carrier with NO cache would have to be replayed, and one with a
/// FITTED cache is an approximation; both are refused by name.
pub(super) fn convert(geometry: &SurfaceGeometry, context: &Context<'_>) -> Result<Oriented> {
    match geometry {
        SurfaceGeometry::Solved(solved) => solved_surface(solved, context),
        SurfaceGeometry::Procedural {
            construction,
            cache: Some(cache),
        } => {
            let construction = context.procedural_surface(construction.as_str())?;
            if construction.cache_fit_tolerance().is_some() {
                return Err(unsupported("procedural (fitted cache)", context));
            }
            solved_surface(cache, context)
        }
        SurfaceGeometry::Procedural { cache: None, .. } => {
            Err(unsupported("procedural (no solved cache)", context))
        }
    }
}

fn solved_surface(geometry: &SolvedSurfaceGeometry, context: &Context<'_>) -> Result<Oriented> {
    match geometry {
        SolvedSurfaceGeometry::Plane(plane) => {
            let frame = frame::frame(plane.origin(), plane.frame());
            // Three points, not a normal: `Plane` stores its own u and v axes, and
            // taking v as `axis x u` keeps the plane's normal equal to the source
            // normal. Swapping the two would flip every face on it.
            Ok(Oriented::forward(Surface::Plane(Plane::new(
                frame.origin,
                frame.origin + frame.reference,
                frame.origin + frame.co_reference(),
            ))))
        }

        SolvedSurfaceGeometry::Cylinder(cylinder) => {
            let frame = frame::frame(cylinder.origin(), cylinder.frame());
            // Positive by type (`PositiveLength`).
            let radius = cylinder.radius().get();
            // A cylinder is a line at distance `radius` from the axis, revolved.
            // The profile is UNIT length: the surface is unbounded along its axis
            // and the profile's own range is incidental, which is exactly what
            // STEP's `CYLINDRICAL_SURFACE` also produces. Consumers that need the
            // real extent pass the face's trim rectangle to
            // `try_into_homogeneous_bspline_surface_over`.
            let base = frame.origin + frame.reference * radius;
            Ok(Oriented::forward(revolution(
                Line(base, base + frame.axis),
                frame,
            )))
        }

        SolvedSurfaceGeometry::Cone(cone) => {
            // An ELLIPTICAL cone is not a surface of revolution, so it has no
            // `RevolutionSurface` form and no other exact one either. Refused by
            // name rather than silently converted into the circular cone that
            // `ratio == 1` would have meant.
            let ratio = cone.ratio().get();
            if (ratio - 1.0).abs() > f64::EPSILON.sqrt() {
                return Err(unsupported("elliptical cone", context));
            }
            let frame = frame::frame(cone.origin(), cone.frame());
            // Nonnegative by type. ZERO is admitted since cadmpeg 0.6 and is a
            // cone posed at its apex: the profile below then starts ON the axis,
            // which is the same line the representation evaluates.
            let radius = cone.radius().get();
            let half_angle = cone.half_angle().get();
            // The representation's chart is `radius + v * tan(half_angle)` at
            // signed axial distance `v`, for EVERY finite angle, so the profile
            // follows that formula and no sign is normalised away.
            let taper = half_angle.tan();
            if !taper.is_finite() {
                return Err(context.malformed(format!(
                    "cone half-angle {half_angle} radians has no finite taper, so its profile is \
                     parallel to its own axis"
                )));
            }
            let base = frame.origin + frame.reference * radius;
            let apex_ward = frame.axis + frame.reference * taper;
            Ok(Oriented::forward(revolution(
                Line(base, base + apex_ward),
                frame,
            )))
        }

        SolvedSurfaceGeometry::Sphere(sphere) => {
            // The frame is deliberately dropped. It fixes the sphere's
            // parameterization, not its point set, and a sphere's point set is
            // invariant under rotation about its own centre. Nothing downstream
            // reads the representation's parameter-space curves --
            // `CompressedFace` has nowhere to store them and every consumer
            // re-derives (u, v) by projecting the 3D boundary -- so any valid
            // parameterization of the right point set is the right answer.
            //
            // The radius is SIGNED (`NonZeroLength`). A negative radius is the
            // same point set as its magnitude, reached through the antipodal
            // chart, whose natural normal points inward: see the module note.
            // The magnitude builds the sphere and the sign becomes `reversed`.
            let signed = sphere.radius().get();
            let analytic = Processor::new(Sphere::new(
                frame::finite_point(sphere.center()),
                signed.abs(),
            ));
            admissible(&analytic, "sphere", context)?;
            Ok(Oriented {
                surface: Surface::SphericalSurface(analytic),
                reversed: signed < 0.0,
            })
        }

        SolvedSurfaceGeometry::Torus(torus) => {
            // Unlike a sphere, a torus's point set DOES depend on its axis, so
            // the frame is carried in the `Processor`'s transform and the entity
            // stays canonical.
            let frame = frame::frame(torus.center(), torus.frame());
            let major = torus.major_radius().get();
            // SIGNED since cadmpeg 0.6 (`NonZeroLength`). A negative tube radius
            // is the same point set as its magnitude with the tube angle shifted
            // by half a turn, and its natural normal still points out of the
            // tube -- see the module note -- so the magnitude is the whole
            // mapping and nothing flips.
            let minor = torus.minor_radius().get().abs();
            // `Torus::new` PANICS on a non-positive radius. Both magnitudes are
            // positive by type, so this call cannot reach the panic.
            let analytic = Processor::with_transform(
                Torus::new(Point3::new(0.0, 0.0, 0.0), major, minor),
                frame.placement(1.0),
            );
            admissible(&analytic, "torus", context)?;
            Ok(Oriented::forward(Surface::ToroidalSurface(analytic)))
        }

        SolvedSurfaceGeometry::Nurbs(surface) => nurbs(surface, context),

        SolvedSurfaceGeometry::Transformed(placed) => {
            let matrix = frame::matrix(placed.transform());
            let determinant = matrix.determinant();
            if !determinant.is_finite() || determinant == 0.0 {
                return Err(crate::Error::UnplaceableOccurrence {
                    format: context.format,
                    kind: "singular placement",
                });
            }
            // A placement maps the carrier's partials by the linear map, and
            // monstertruck's `Processor::normal` divides its normal by the
            // determinant. A NEGATIVE determinant therefore flips the
            // transformed surface's natural normal relative to the oriented
            // normal the source stated: `reversed` must absorb that sign, or a
            // mirrored carrier is handed on facing inward.
            let mut oriented = solved_surface(placed.basis(), context)?;
            if determinant < 0.0 {
                oriented.reversed = !oriented.reversed;
            }
            oriented.surface.transform_by(matrix);
            Ok(oriented)
        }

        // Refused by name, every one of them. `Polygonal` is an approximation
        // with a chordal bound, and admitting it as a spline patch would present
        // a tessellation as exact geometry; `Unknown` is bytes the decoder itself
        // could not classify.
        SolvedSurfaceGeometry::Polygonal(_) => Err(unsupported("source-native polygonal", context)),
        SolvedSurfaceGeometry::Unknown { .. } => Err(unsupported("unclassified native", context)),
        // Matched EXHAUSTIVELY, with no catch-all. A cadmpeg release that adds a
        // carrier breaks this build -- which is what we want. A catch-all would
        // turn that release into a silent runtime refusal for a surface someone
        // then has to discover is unsupported, instead of a decision made when
        // the dependency moves.
    }
}

/// Revolve a profile line about a frame's axis, facing AWAY from the axis.
///
/// A line revolved about an axis it runs alongside has its natural normal
/// pointing TOWARD the axis, which is the opposite of the representation's
/// cylinder and cone charts (`Su x Sv` points away from the axis wherever the
/// local radius is positive). So the revolution is inverted, exactly as
/// monstertruck's own STEP reader does for `CYLINDRICAL_SURFACE` and
/// `CONICAL_SURFACE` (`step_types/surface.rs`), and the two readers hand the
/// kernel the same surface. Until cadmpeg 0.6 this inversion was missing here,
/// and every cylindrical and conical face this bridge produced faced inward;
/// `cylinders_and_cones_face_like_the_source` pins it against cadmpeg's own
/// evaluator.
fn revolution(profile: Line<Point3>, frame: frame::Frame) -> Surface {
    let mut surface = Processor::new(RevolutionSurface::by_revolution(
        Curve::Line(profile),
        frame.origin,
        frame.axis,
    ));
    surface.invert();
    Surface::RevolutionSurface(surface)
}

/// Ask the net builder whether this analytic carrier is admissible.
///
/// See the module note: the builder's refusal IS the predicate, so probing it
/// cannot drift from it the way a restated guard can.
fn admissible<T: TryIntoHomogeneousBsplineSurface>(
    surface: &T,
    what: &'static str,
    context: &Context<'_>,
) -> Result<()> {
    match surface.try_into_homogeneous_bspline_surface() {
        Some(_) => Ok(()),
        None => Err(crate::Error::UnsupportedSurfaceKind {
            format: context.format,
            kind: match what {
                "torus" => "degenerate torus",
                other => other,
            },
        }),
    }
}

fn unsupported(kind: &'static str, context: &Context<'_>) -> crate::Error {
    crate::Error::UnsupportedSurfaceKind {
        format: context.format,
        kind,
    }
}

/// A free-form surface, rational or not.
///
/// Non-rational nets become [`Surface::BsplineSurface`] rather than a
/// [`Surface::NurbsSurface`] with every weight at one: the polynomial form has no
/// weight division in its evaluation, and the boolean kernel's exact-patch
/// machinery distinguishes the two.
///
/// # Periodic and unclamped knot vectors
///
/// The representation stores every net, periodic or not, as a FULL knot vector
/// with every pole, and evaluates it over the effective domain
/// `[knot[degree], knot[count]]` only (`eval::nurbs_pcurve_parameter_domain`).
/// A periodic net additionally wraps its parameter modulo that domain's width.
/// So both are exactly convertible: the spline over the effective domain is the
/// whole carrier, and a periodic one is that same spline traversed once. The
/// net is clamped onto the effective domain by knot insertion -- which leaves
/// the spline unchanged -- so the result is an ordinary clamped net whose range
/// is the source's domain. Nothing past the domain survives: a clamp at the
/// FULL knot extent would keep the extrapolation spans, which are not the
/// carrier.
fn nurbs(surface: &IrNurbsSurface, context: &Context<'_>) -> Result<Oriented> {
    if surface.cache_fit_tolerance().is_some() {
        // The net is a FIT to some other carrier. Admitting it would present an
        // approximation as the exact surface. This shares
        // `UnsupportedSurfaceKind` with the construction-level fitted cache so
        // that one condition does not change error class depending on where the
        // tolerance was recorded.
        return Err(unsupported("fitted NURBS surface", context));
    }
    let knots = (
        KnotVector::from(surface.u_knots().to_vec()),
        KnotVector::from(surface.v_knots().to_vec()),
    );
    let degrees = (surface.u_degree() as usize, surface.v_degree() as usize);
    // Rows are u, columns are v -- `control_grid()`'s outer index is u -- which
    // is the row-of-v layout `BsplineSurface` wants.
    let grid = surface.control_grid();
    let converted = match surface.weights() {
        None => {
            let net = grid
                .iter()
                .map(|row| row.iter().map(|pole| frame::finite_point(*pole)).collect())
                .collect();
            let net = built(BsplineSurface::try_new(knots, net), context)?;
            Surface::BsplineSurface(clamped_onto_domain(net, degrees, context)?)
        }
        Some(weights) => {
            let net = grid
                .iter()
                .zip(&weights)
                .map(|(row, weights)| {
                    row.iter()
                        .zip(weights)
                        .map(|(pole, weight)| homogeneous(&pole.get(), weight.get()))
                        .collect()
                })
                .collect();
            let net = built(BsplineSurface::try_new(knots, net), context)?;
            Surface::NurbsSurface(NurbsSurface::new(clamped_onto_domain(
                net, degrees, context,
            )?))
        }
    };
    Ok(Oriented {
        surface: converted,
        // `Pu x Pv` with u the row index is monstertruck's natural normal too, so
        // the source's own flag is the whole answer.
        reversed: surface.normal_reversed(),
    })
}

/// Clamp a net onto its effective domain `[knot[degree], knot[count]]` in both
/// directions, leaving the spline unchanged. A net already clamped is returned
/// as it was, so every clamped source converts exactly as before.
fn clamped_onto_domain<P: ControlPoint<f64> + Tolerance>(
    mut net: BsplineSurface<P>,
    (u_degree, v_degree): (usize, usize),
    context: &Context<'_>,
) -> Result<BsplineSurface<P>> {
    let domain = |knots: &KnotVector, count: usize, degree: usize| {
        (knots[degree], knots[count], {
            let front = knots[..=degree].iter().all(|knot| *knot == knots[degree]);
            let back = knots[count..].iter().all(|knot| *knot == knots[count]);
            (front, back)
        })
    };
    let (u_low, u_high, (u_front, u_back)) =
        domain(net.knot_vector_u(), net.control_points().len(), u_degree);
    if !u_back {
        let _beyond = net.cut_u(u_high);
    }
    if !u_front {
        net = net.cut_u(u_low);
    }
    let (v_low, v_high, (v_front, v_back)) =
        domain(net.knot_vector_v(), net.control_points()[0].len(), v_degree);
    if !v_back {
        let _beyond = net.cut_v(v_high);
    }
    if !v_front {
        net = net.cut_v(v_low);
    }
    // Read back what the cuts produced rather than trusting them: `cut_u`
    // snaps a parameter near a domain end onto it, and a net whose range moved
    // is not the carrier.
    let (u_now, v_now) = (
        domain(net.knot_vector_u(), net.control_points().len(), u_degree),
        domain(net.knot_vector_v(), net.control_points()[0].len(), v_degree),
    );
    let range_holds = (u_now.0, u_now.1, v_now.0, v_now.1) == (u_low, u_high, v_low, v_high);
    if !net.is_clamped() || !range_holds {
        return Err(context.malformed(format!(
            "a NURBS surface's knot vectors could not be clamped onto their effective domain \
             [{u_low}, {u_high}] x [{v_low}, {v_high}]"
        )));
    }
    Ok(net)
}

/// A rational control point in homogeneous coordinates.
pub(super) fn homogeneous(point: &cadmpeg_ir::math::Point3, weight: f64) -> Vector4 {
    Vector4::new(point.x * weight, point.y * weight, point.z * weight, weight)
}

/// Turn a geometry-crate construction failure into a typed import failure.
///
/// `BsplineSurface::new` panics where `try_new` reports; on file input the
/// reporting one is the only defensible choice.
pub(super) fn built<T>(
    result: std::result::Result<T, monstertruck_geometry::errors::Error>,
    context: &Context<'_>,
) -> Result<T> {
    result.map_err(|error| context.malformed(error.to_string()))
}
