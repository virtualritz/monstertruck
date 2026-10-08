use algo::curve::{presearch_closest_point, search_closest_parameter};
use monstertruck_geometry::prelude::*;
use std::iter::{once, repeat_n};

use crate::HashSet;

use super::error::FilletError;
use super::topology::*;
use super::types::*;

type Result<T> = std::result::Result<T, FilletError>;

const MITER_SPANS: usize = 32;
const SMOOTH_COSINE: f64 = 0.999_999;
const MEET_TOLERANCE: f64 = 1.0e-4;

fn failed(context: &'static str) -> FilletError { FilletError::GeometryFailed { context } }

fn is_smooth(incoming: &Edge, outgoing: &Edge) -> bool {
    let incoming = incoming.oriented_curve();
    let outgoing = outgoing.oriented_curve();
    let end = incoming.der(incoming.range_tuple().1).normalize();
    let start = outgoing.der(outgoing.range_tuple().0).normalize();
    end.dot(start) > SMOOTH_COSINE
}

struct Junction {
    shared: Vertex,
    side: Vertex,
    edge: Edge,
    incoming: (f64, f64),
    outgoing: (f64, f64),
}

fn closest<C0, C1>(curve0: &C0, curve1: &C1) -> Option<(f64, f64)>
where
    C0: ParametricCurve3D + BoundedCurve,
    C1: ParametricCurve3D + BoundedCurve, {
    let hint = presearch_closest_point(
        curve0,
        curve1,
        (curve0.range_tuple(), curve1.range_tuple()),
        50,
    );
    let (t0, t1) = search_closest_parameter(curve0, curve1, hint, 100)?;
    (curve0.subs(t0).distance(curve1.subs(t1)) < MEET_TOLERANCE).then_some((t0, t1))
}

fn signed_distance(
    point: Point3,
    surface: &NurbsSurface<Vector4>,
    hint: &mut Option<(f64, f64)>,
) -> Option<f64> {
    let (u, v) = surface.search_nearest_parameter(point, *hint, 100)?;
    *hint = Some((u, v));
    Some((point - surface.subs(u, v)).dot(surface.normal(u, v)))
}

fn meet_along_row(
    surface: &NurbsSurface<Vector4>,
    u: f64,
    guess: f64,
    other: &NurbsSurface<Vector4>,
    hint: &mut Option<(f64, f64)>,
) -> Option<f64> {
    let (_, (v_low, v_high)) = surface.range_tuple();
    let mut f = |v: f64| signed_distance(surface.subs(u, v), other, hint);
    let (mut v0, mut v1) = (guess, guess + (v_high - v_low) * 1.0e-3);
    let (mut f0, mut f1) = (f(v0)?, f(v1)?);
    for _ in 0..64 {
        if f1.abs() < TOLERANCE * 1.0e-3 {
            return Some(v1);
        }
        let slope = f1 - f0;
        if slope.abs() < f64::MIN_POSITIVE {
            return None;
        }
        let v2 = v1 - f1 * (v1 - v0) / slope;
        (v0, f0, v1) = (v1, f1, v2);
        f1 = f(v1)?;
    }
    (f1.abs() < TOLERANCE).then_some(v1)
}

fn interpolate(points: Vec<Point3>) -> Option<NurbsCurve<Vector4>> {
    let count = points.len();
    let degree = 3.min(count - 1);
    let lengths: Vec<f64> = points.windows(2).map(|w| w[0].distance(w[1])).collect();
    let total: f64 = lengths.iter().sum();
    let parameters: Vec<f64> = once(0.0)
        .chain(lengths.iter().scan(0.0, |run, length| {
            *run += length / total;
            Some(*run)
        }))
        .collect();
    let knots: Vec<f64> = repeat_n(0.0, degree + 1)
        .chain(
            (1..count - degree)
                .map(|j| parameters[j..j + degree].iter().sum::<f64>() / degree as f64),
        )
        .chain(repeat_n(1.0, degree + 1))
        .collect();
    let parameter_points: Vec<(f64, Point3)> = parameters.into_iter().zip(points).collect();
    BsplineCurve::try_interpolate(KnotVector::from(knots), parameter_points)
        .ok()
        .map(NurbsCurve::from)
}

fn trim(mut curve: NurbsCurve<Vector4>, t0: f64, t1: f64) -> NurbsCurve<Vector4> {
    let (low, high) = curve.range_tuple();
    if t1 < high - TOLERANCE {
        curve.cut(t1);
    }
    if t0 > low + TOLERANCE {
        curve = curve.cut(t0);
    }
    curve
}

fn seam_at(side_face: &Face, wire_edge: &Edge) -> Result<Edge> {
    let corner = wire_edge.front();
    side_face
        .edge_iter()
        .find(|edge| !edge.is_same(wire_edge) && (edge.front() == corner || edge.back() == corner))
        .ok_or(failed("seam edge at chain junction"))
}

fn cut_seam(
    seam: &Edge,
    corner: &Vertex,
    point: Point3,
    replacements: &mut EdgeReplacements,
) -> Result<Vertex> {
    let current = replacements.current(seam);
    let curve = current.oriented_curve();
    let t = curve
        .search_nearest_parameter(point, None, 100)
        .ok_or(failed("project contact point onto seam"))?;
    let vertex = Vertex::new(curve.subs(t));
    let (head, tail) = current
        .not_strictly_cut(&vertex)
        .ok_or(failed("cut seam edge at contact point"))?;
    replacements.insert(
        seam,
        if current.front() == corner {
            tail
        } else {
            head
        },
    );
    Ok(vertex)
}

fn smooth_junction(
    incoming: &NurbsSurface<Vector4>,
    outgoing: &NurbsSurface<Vector4>,
    seam: &Edge,
    corner: &Vertex,
    replacements: &mut EdgeReplacements,
) -> Result<Junction> {
    let last_row = outgoing.control_points().len() - 1;
    let (_, (_, incoming_end)) = incoming.range_tuple();
    let (_, (outgoing_start, _)) = outgoing.range_tuple();
    let shared = Vertex::new(outgoing.curve_v(0).front());
    let side = cut_seam(
        seam,
        corner,
        outgoing.curve_v(last_row).front(),
        replacements,
    )?;
    let edge = Edge::new(&shared, &side, outgoing.curve_u(0).into());
    Ok(Junction {
        shared,
        side,
        edge,
        incoming: (incoming_end, incoming_end),
        outgoing: (outgoing_start, outgoing_start),
    })
}

fn miter_junction(
    incoming: &NurbsSurface<Vector4>,
    outgoing: &NurbsSurface<Vector4>,
    seam: &Edge,
    corner: &Vertex,
    replacements: &mut EdgeReplacements,
) -> Result<Junction> {
    let last_row = incoming.control_points().len() - 1;
    let not_meeting = || failed("fillets do not meet at corner");
    let (a0, b0) = closest(&incoming.curve_v(0), &outgoing.curve_v(0)).ok_or_else(not_meeting)?;
    let seam_curve = seam.oriented_curve();
    let (a1, seam_a) = closest(&incoming.curve_v(last_row), &seam_curve).ok_or_else(not_meeting)?;
    let (b1, seam_b) = closest(&outgoing.curve_v(last_row), &seam_curve).ok_or_else(not_meeting)?;

    let shared_point = incoming
        .curve_v(0)
        .subs(a0)
        .midpoint(outgoing.curve_v(0).subs(b0));
    let side_point = seam_curve.subs(seam_a).midpoint(seam_curve.subs(seam_b));
    let shared = Vertex::new(shared_point);
    let side = cut_seam(seam, corner, side_point, replacements)?;

    let ((u_low, u_high), _) = incoming.range_tuple();
    let mut hint = None;
    let interior = (1..MITER_SPANS)
        .map(|j| {
            let s = j as f64 / MITER_SPANS as f64;
            let u = u_low + (u_high - u_low) * s;
            let guess = a0 + (a1 - a0) * s;
            meet_along_row(incoming, u, guess, outgoing, &mut hint).map(|v| incoming.subs(u, v))
        })
        .collect::<Option<Vec<_>>>()
        .ok_or_else(not_meeting)?;
    let points = once(shared_point)
        .chain(interior)
        .chain(once(side.point()))
        .collect();
    let curve = interpolate(points).ok_or(failed("interpolate miter curve"))?;
    let edge = Edge::new(&shared, &side, curve.into());
    Ok(Junction {
        shared,
        side,
        edge,
        incoming: (a0, a1),
        outgoing: (b0, b1),
    })
}

fn average_seam(previous: &mut NurbsSurface<Vector4>, next: &mut NurbsSurface<Vector4>) {
    (0..next.control_points().len()).for_each(|j| {
        let len = previous.control_points()[j].len();
        let p = *previous.control_point(j, len - 1);
        let q = *next.control_point(j, 0);
        let c = (p + q) / 2.0;
        *previous.control_point_mut(j, len - 1) = c;
        *next.control_point_mut(j, 0) = c;
    });
}

fn end_junction(
    surface: &NurbsSurface<Vector4>,
    shared_neighbor: &Edge,
    side_neighbor: &Edge,
    corner: &Vertex,
    end_surface: &NurbsSurface<Vector4>,
    cuts: &mut EdgeReplacements,
) -> Result<Junction> {
    let last_row = surface.control_points().len() - 1;
    let not_meeting = || failed("fillet does not reach the end face");
    let (t0, _) =
        closest(&surface.curve_v(0), &shared_neighbor.oriented_curve()).ok_or_else(not_meeting)?;
    let (t1, _) = closest(&surface.curve_v(last_row), &side_neighbor.oriented_curve())
        .ok_or_else(not_meeting)?;
    let shared = cut_seam(shared_neighbor, corner, surface.curve_v(0).subs(t0), cuts)?;
    let side = cut_seam(
        side_neighbor,
        corner,
        surface.curve_v(last_row).subs(t1),
        cuts,
    )?;

    let ((u_low, u_high), _) = surface.range_tuple();
    let mut hint = None;
    let interior = (1..MITER_SPANS)
        .map(|j| {
            let s = j as f64 / MITER_SPANS as f64;
            let u = u_low + (u_high - u_low) * s;
            let guess = t0 + (t1 - t0) * s;
            meet_along_row(surface, u, guess, end_surface, &mut hint).map(|v| surface.subs(u, v))
        })
        .collect::<Option<Vec<_>>>()
        .ok_or_else(not_meeting)?;
    let points = once(shared.point())
        .chain(interior)
        .chain(once(side.point()))
        .collect();
    let curve = interpolate(points).ok_or(failed("interpolate end curve"))?;
    let edge = Edge::new(&shared, &side, curve.into());
    Ok(Junction {
        shared,
        side,
        edge,
        incoming: (t0, t1),
        outgoing: (t0, t1),
    })
}

fn face_index_with_edge(shell: &Shell, edge: &Edge, excluded: usize) -> Option<usize> {
    shell
        .face_iter()
        .enumerate()
        .find(|(index, face)| *index != excluded && face.edge_iter().any(|e| e.is_same(edge)))
        .map(|(index, _)| index)
}

fn free_end_junction(
    surface: &NurbsSurface<Vector4>,
    at_start: bool,
    end: &ChainEnd,
    cuts: &mut EdgeReplacements,
) -> Result<Junction> {
    let ((u_low, u_high), (v_low, v_high)) = surface.range_tuple();
    let v = if at_start { v_low } else { v_high };
    let column = if at_start {
        0
    } else {
        surface.control_points()[0].len() - 1
    };
    let shared = cut_seam(
        &end.shared_neighbor,
        &end.corner,
        surface.subs(u_low, v),
        cuts,
    )?;
    let side = cut_seam(
        &end.side_neighbor,
        &end.corner,
        surface.subs(u_high, v),
        cuts,
    )?;
    let edge = Edge::new(&shared, &side, surface.curve_u(column).into());
    Ok(Junction {
        shared,
        side,
        edge,
        incoming: (v, v),
        outgoing: (v, v),
    })
}

struct ChainEnd {
    corner: Vertex,
    shared_neighbor: Edge,
    side_neighbor: Edge,
    end_face: Option<usize>,
}

fn chain_end(
    shell: &Shell,
    wire_edge: &Edge,
    at_start: bool,
    shared_face: usize,
    side_face: usize,
) -> Result<ChainEnd> {
    let corner = if at_start {
        wire_edge.front()
    } else {
        wire_edge.back()
    }
    .clone();
    let (front, back) = find_adjacent_edge(&shell[shared_face], wire_edge.id())
        .ok_or(failed("neighbor edge on shared face"))?;
    let shared_neighbor = if at_start { front } else { back };
    let side_neighbor = shell[side_face]
        .edge_iter()
        .find(|edge| {
            !edge.is_same(wire_edge) && (edge.front() == &corner || edge.back() == &corner)
        })
        .ok_or(failed("neighbor edge on side face"))?;
    let end_face = face_index_with_edge(shell, &shared_neighbor, shared_face);
    Ok(ChainEnd {
        corner,
        shared_neighbor,
        side_neighbor,
        end_face,
    })
}

fn insert_between(face: &Face, edge: &Edge) -> Option<Face> {
    let (a, b) = (edge.front(), edge.back());
    let mut boundaries = face.absolute_boundaries().clone();
    let inserted = boundaries.iter_mut().any(|boundary| {
        let len = boundary.len();
        (0..len)
            .find_map(|i| {
                let (current, following) = (&boundary[i], &boundary[(i + 1) % len]);
                if current.back() == b && following.front() == a {
                    Some((i + 1, edge.inverse()))
                } else if current.back() == a && following.front() == b {
                    Some((i + 1, edge.clone()))
                } else {
                    None
                }
            })
            .map(|(position, oriented)| boundary.insert(position, oriented))
            .is_some()
    });
    inserted.then(|| {
        let mut new_face = Face::new_unchecked(boundaries, face.surface());
        if !face.orientation() {
            new_face.invert();
        }
        new_face
    })
}

/// Blends every edge of `chain` into `shell`, closing the corners where they meet.
pub(super) fn fillet_chain<R: Fn(f64) -> f64>(
    shell: &mut Shell,
    chain: &Chain<'_, R>,
    closed: bool,
) -> Result<()> {
    let (wire, shared_face_index, adjacent_faces) =
        (chain.wire, chain.shared_face_index, chain.adjacent_faces);
    let n = wire.len();
    let shared_face = shared_face_index.face_index;
    let previous = |k: usize| (k + n - 1) % n;
    let interior = |k: usize| closed || (k > 0 && k < n);
    let smooth: Vec<bool> = (0..=n)
        .map(|k| interior(k) && is_smooth(&wire[previous(k % n)], &wire[k % n]))
        .collect();
    let ends: Vec<ChainEnd> = match closed {
        true => Vec::new(),
        false => vec![
            chain_end(
                shell,
                &wire[0],
                true,
                shared_face,
                adjacent_faces[0].face_index,
            )?,
            chain_end(
                shell,
                &wire[n - 1],
                false,
                shared_face,
                adjacent_faces[n - 1].face_index,
            )?,
        ],
    };
    let free_end = |k: usize| match (closed, k) {
        (false, 0) => ends[0].end_face.is_none(),
        (false, k) if k == n => ends[1].end_face.is_none(),
        _ => false,
    };
    let extensions: Vec<(bool, bool)> = (0..n)
        .map(|k| {
            (
                !smooth[k] && !free_end(k),
                !smooth[k + 1] && !free_end(k + 1),
            )
        })
        .collect();

    let mut surfaces = fillet_surfaces_with_extensions(shell, chain, &extensions)
        .ok_or(FilletError::FilletSurfaceComputationFailed)?;
    (0..n)
        .filter(|&k| smooth[k] && previous(k) != k)
        .for_each(|k| {
            let p = previous(k);
            let (low, high) = surfaces.split_at_mut(p.max(k));
            match p < k {
                true => average_seam(&mut low[p], &mut high[0]),
                false => average_seam(&mut high[0], &mut low[k]),
            }
        });

    let mut cuts = EdgeReplacements::default();
    let mut end_curves: Vec<(usize, Edge)> = Vec::new();
    let junction_count = if closed { n } else { n + 1 };
    let junctions: Vec<Junction> = (0..junction_count)
        .map(|k| {
            if !interior(k) {
                let at_start = k == 0;
                let edge_index = if at_start { 0 } else { n - 1 };
                let end = &ends[if at_start { 0 } else { 1 }];
                let surface = &surfaces[edge_index];
                let Some(end_face) = end.end_face else {
                    return free_end_junction(surface, at_start, end, &mut cuts);
                };
                let junction = end_junction(
                    surface,
                    &end.shared_neighbor,
                    &end.side_neighbor,
                    &end.corner,
                    &shell[end_face].oriented_surface(),
                    &mut cuts,
                )?;
                end_curves.push((end_face, junction.edge.clone()));
                return Ok(junction);
            }
            let k = k % n;
            let seam = seam_at(&shell[adjacent_faces[k].face_index], &wire[k])?;
            let corner = wire[k].front();
            let (incoming, outgoing) = (&surfaces[previous(k)], &surfaces[k]);
            match smooth[k] {
                true => smooth_junction(incoming, outgoing, &seam, corner, &mut cuts),
                false => miter_junction(incoming, outgoing, &seam, corner, &mut cuts),
            }
        })
        .collect::<Result<_>>()?;

    let following = |k: usize| (k + 1) % junction_count;
    let last_row = surfaces[0].control_points().len() - 1;
    let segment = |k: usize, row: usize, pick: fn(&(f64, f64)) -> f64| {
        let (start, end) = (&junctions[k], &junctions[following(k)]);
        trim(
            surfaces[k].curve_v(row),
            pick(&start.outgoing),
            pick(&end.incoming),
        )
    };
    let shared_edges: Vec<Edge> = (0..n)
        .map(|k| {
            let curve = segment(k, 0, |params| params.0);
            Edge::new(
                &junctions[k].shared,
                &junctions[following(k)].shared,
                curve.into(),
            )
        })
        .collect();
    let side_edges: Vec<Edge> = (0..n)
        .map(|k| {
            let curve = segment(k, last_row, |params| params.1);
            Edge::new(
                &junctions[k].side,
                &junctions[following(k)].side,
                curve.into(),
            )
        })
        .collect();

    let mut shared_replacements = EdgeReplacements::default();
    let mut side_replacements = EdgeReplacements::default();
    (0..n).for_each(|k| {
        shared_replacements.insert(&wire[k], shared_edges[k].clone());
        side_replacements.insert(&wire[k], side_edges[k].clone());
    });

    let fillet_faces: Vec<Face> = (0..n)
        .map(|k| {
            let boundary: Wire = vec![
                shared_edges[k].inverse(),
                junctions[k].edge.clone(),
                side_edges[k].clone(),
                junctions[following(k)].edge.inverse(),
            ]
            .into();
            Face::new_unchecked(vec![boundary], surfaces[k].clone())
        })
        .collect();

    let side_faces: HashSet<usize> = adjacent_faces
        .iter()
        .map(|index| index.face_index)
        .collect();
    (0..shell.len()).for_each(|face| {
        let mut updated = cuts.apply(&shell[face]);
        if face == shared_face {
            updated = shared_replacements.apply(&updated);
        } else if side_faces.contains(&face) {
            updated = side_replacements.apply(&updated);
        }
        shell[face] = updated;
    });
    end_curves.iter().try_for_each(|(face, edge)| {
        shell[*face] = insert_between(&shell[*face], edge).ok_or(failed("insert end curve"))?;
        Ok::<(), FilletError>(())
    })?;
    shell.extend(fillet_faces);
    Ok(())
}
