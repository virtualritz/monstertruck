use monstertruck_geometry::prelude::*;

use super::chain::fillet_chain;
use super::error::FilletError;
use super::geometry::*;
use super::params::{FilletOptions, FilletProfile, RadiusSpec};
use super::topology::*;
use super::types::*;

type Result<T> = std::result::Result<T, FilletError>;

/// Fillets a single shared edge between two faces.
///
/// Returns `(new_face0, new_face1, fillet_face)`.
pub fn fillet(
    face0: &Face,
    face1: &Face,
    filleted_edge_id: EdgeId,
    options: &FilletOptions,
) -> Result<(Face, Face, Face)> {
    // The relay-sphere construction picks its cutting side from the seam
    // direction relative to the two surface normals. Boolean output reaches
    // both boundary-winding conventions, so when the natural-side attempt
    // fails geometrically, retry with the sphere side flipped.
    fillet_on_side(face0, face1, filleted_edge_id, options, false)
        .or_else(|_| fillet_on_side(face0, face1, filleted_edge_id, options, true))
}

fn fillet_on_side(
    face0: &Face,
    face1: &Face,
    filleted_edge_id: EdgeId,
    options: &FilletOptions,
    flip_side: bool,
) -> Result<(Face, Face, Face)> {
    let is_filleted_edge = move |edge: &Edge| edge.id() == filleted_edge_id;
    let filleted_edge =
        face0
            .edge_iter()
            .find(is_filleted_edge)
            .ok_or(FilletError::GeometryFailed {
                context: "filleted edge not found in face0",
            })?;

    let division = options.divisions.get();
    let surface0 = face0.oriented_surface();
    let surface1 = face1.oriented_surface();
    let fillet_surface = {
        let curve = filleted_edge.oriented_curve();
        let make_with_extend = |radius: &dyn Fn(f64) -> f64, extend: bool| match &options.profile {
            FilletProfile::Round => rolling_ball_fillet_surface(
                &surface0, &surface1, &curve, division, radius, extend, flip_side,
            ),
            FilletProfile::Chamfer => chamfer_fillet_surface(
                &surface0, &surface1, &curve, division, radius, extend, flip_side,
            ),
            FilletProfile::Ridge => ridge_fillet_surface(
                &surface0, &surface1, &curve, division, radius, extend, flip_side,
            ),
            FilletProfile::Custom(profile) => custom_fillet_surface(
                &surface0, &surface1, &curve, division, radius, extend, flip_side, profile,
            ),
        };
        let make = |radius: &dyn Fn(f64) -> f64| {
            make_with_extend(radius, true).or_else(|| make_with_extend(radius, false))
        };
        match &options.radius {
            RadiusSpec::Constant(r) => {
                let r = *r;
                make(&|_| r)
            }
            RadiusSpec::Variable(f) => make(f.as_ref()),
            RadiusSpec::PerEdge(radii) => match radii.first() {
                Some(&r) => make(&|_| r),
                None => None,
            },
        }
        .ok_or(FilletError::GeometryFailed {
            context: "fillet surface computation",
        })?
    };

    let (new_face0, fillet_edge0) = {
        let bezier = fillet_surface.curve_v(0);
        cut_face_by_bezier(face0, bezier, filleted_edge_id).ok_or(FilletError::GeometryFailed {
            context: "cut face0 by bezier",
        })?
    };
    let (new_face1, fillet_edge1) = {
        let bezier = fillet_surface.curve_v(fillet_surface.control_points().len() - 1);
        cut_face_by_bezier(face1, bezier.inverse(), filleted_edge_id).ok_or(
            FilletError::GeometryFailed {
                context: "cut face1 by bezier",
            },
        )?
    };

    let ((v0, v1), (v2, v3)) = (fillet_edge0.ends(), fillet_edge1.ends());
    let edge0 = create_pcurve_edge((v0, (0.0, 0.0)), (v3, (1.0, 0.0)), &fillet_surface).ok_or(
        FilletError::GeometryFailed {
            context: "create pcurve edge0",
        },
    )?;
    let edge1 = create_pcurve_edge((v2, (1.0, 1.0)), (v1, (0.0, 1.0)), &fillet_surface).ok_or(
        FilletError::GeometryFailed {
            context: "create pcurve edge1",
        },
    )?;
    let fillet = {
        let fillet_boundary = [fillet_edge0.inverse(), edge0, fillet_edge1.inverse(), edge1];
        Face::new_unchecked(vec![fillet_boundary.into()], fillet_surface)
    };

    Ok((new_face0, new_face1, fillet))
}

/// Fillets a shared edge between two faces, optionally updating adjacent side faces.
///
/// Returns `(new_face0, new_face1, fillet_face, new_side0, new_side1)`.
#[allow(clippy::type_complexity)]
pub fn fillet_with_side(
    face0: &Face,
    face1: &Face,
    filleted_edge_id: EdgeId,
    side0: Option<&Face>,
    side1: Option<&Face>,
    options: &FilletOptions,
) -> Result<(Face, Face, Face, Option<Face>, Option<Face>)> {
    let (new_face0, new_face1, fillet) = fillet(face0, face1, filleted_edge_id, options)?;

    let (front_edge0, back_edge0) = {
        let fillet_edge_id = fillet.absolute_boundaries()[0][0].id();
        find_adjacent_edge(&new_face0, fillet_edge_id).ok_or(FilletError::GeometryFailed {
            context: "find adjacent edge in new_face0",
        })?
    };
    let (front_edge1, back_edge1) = {
        let fillet_edge_id = fillet.absolute_boundaries()[0][2].id();
        find_adjacent_edge(&new_face1, fillet_edge_id).ok_or(FilletError::GeometryFailed {
            context: "find adjacent edge in new_face1",
        })?
    };

    let is_filleted_edge = |edge: &Edge| edge.id() == filleted_edge_id;
    let filleted_edge =
        face0
            .edge_iter()
            .find(is_filleted_edge)
            .ok_or(FilletError::GeometryFailed {
                context: "filleted edge not found in face0",
            })?;
    let (v0, v1) = filleted_edge.ends();

    let new_side0 = side0.and_then(|side0| {
        let fillet_edge = &fillet.absolute_boundaries()[0][1];
        create_new_side(side0, fillet_edge, v0.id(), &front_edge0, &back_edge1)
    });
    let new_side1 = side1.and_then(|side1| {
        let fillet_edge = &fillet.absolute_boundaries()[0][3];
        create_new_side(side1, fillet_edge, v1.id(), &front_edge1, &back_edge0)
    });
    Ok((new_face0, new_face1, fillet, new_side0, new_side1))
}

/// Fillets along a wire of edges sharing a common face in the shell.
///
/// Supports both open and closed wires. Modifies `shell` in place by replacing
/// filleted faces and adding new fillet faces.
pub fn fillet_along_wire(shell: &mut Shell, wire: &Wire, options: &FilletOptions) -> Result<()> {
    let division = options.divisions.get();

    // Validate variable radius constraint for closed wire fillets.
    // Open wires don't wrap around, so f(0) ≈ f(1) is only needed for closed wires.
    if wire.is_closed()
        && let RadiusSpec::Variable(f) = &options.radius
        && !f(0.0).near2(&f(1.0))
    {
        return Err(FilletError::VariableRadiusUnsupported);
    }
    if !wire.is_continuous() {
        return Err(FilletError::DiscontinuousWire);
    }

    let closed = wire.is_closed();

    let shared_face_index =
        find_shared_face_with_front_edge(shell, wire).ok_or(FilletError::SharedFaceNotFound)?;
    let adjacent_faces = enumerate_adjacent_faces(shell, wire, shared_face_index)
        .ok_or(FilletError::AdjacentFacesNotFound)?;

    let radius: Box<dyn Fn(f64) -> f64 + '_> = match &options.radius {
        RadiusSpec::Constant(r) => {
            let r = *r;
            Box::new(move |_| r)
        }
        RadiusSpec::Variable(f) => Box::new(f.as_ref()),
        RadiusSpec::PerEdge(radii) => {
            if radii.len() != wire.len() {
                return Err(FilletError::PerEdgeRadiusMismatch {
                    given: radii.len(),
                    expected: wire.len(),
                });
            }
            let (starts, spans) = wire_edge_starts_and_spans(wire)
                .ok_or(FilletError::FilletSurfaceComputationFailed)?;
            let ends: Vec<f64> = starts
                .iter()
                .zip(spans.iter())
                .map(|(start, span)| start + span)
                .collect();
            Box::new(move |t: f64| {
                let global_t = t.clamp(0.0, 1.0);
                let index = ends
                    .partition_point(|&end| end < global_t)
                    .min(radii.len() - 1);
                radii[index]
            })
        }
    };

    let chain = Chain {
        wire,
        shared_face_index,
        adjacent_faces: &adjacent_faces,
        radius,
        division,
        profile: &options.profile,
    };
    fillet_chain(shell, &chain, closed)
}
