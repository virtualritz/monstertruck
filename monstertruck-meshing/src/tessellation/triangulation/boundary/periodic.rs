//! Periodic parameter domains: wraparound normalisation, seam opening and
//! alignment, and the singular-axis (pole) closures that make a periodic loop
//! usable in 2D.
//!
//! Split out of the module file so the source stays readable; the module path
//! is unchanged.

use super::*;

const PERIODIC_LOOP_OTHER_AXIS_TOLERANCE: f64 = 1.0e-6;
const SINGULAR_RADIUS_RELATIVE_TOLERANCE: f64 = 1.0e-6;
const DEFAULT_SINGULAR_PARAMETER_PROBE: f64 = 1.0;

fn abs_diff(previous: f64) -> impl Fn(&f64, &f64) -> std::cmp::Ordering {
    let f = move |x: &f64| f64::abs(x - previous);
    // SAFETY: UV parameters from surface evaluation are finite, so comparison succeeds.
    move |x: &f64, y: &f64| f(x).partial_cmp(&f(y)).unwrap()
}
pub(super) fn periodic_min_difference(u: f64, u0: f64, up: f64) -> f64 {
    let closure = |i| u + i as f64 * up;
    // SAFETY: the iterator (-2..=2) is non-empty, containing five elements.
    (-2..=2).map(closure).min_by(abs_diff(u0)).unwrap()
}

pub(super) fn normalize_range(curve: &mut Vec<SurfacePoint>, compidx: usize, (u0, u1): (f64, f64)) {
    let p = curve[0];
    let q = curve[curve.len() - 1];
    let tmp = f64::min(p[compidx], q[compidx]) + TOLERANCE;
    let del = f64::floor((tmp - u0) / (u1 - u0)) * (u1 - u0);
    curve.iter_mut().for_each(|p| p[compidx] -= del);
    let Some(i) = curve
        .iter()
        .position(|p| (curve[0][compidx] - u1) * (p[compidx] - u1) < 0.0)
    else {
        return;
    };
    let mut curve1 = curve.split_off(i + 1);
    curve1.pop();
    curve1.insert(0, curve[i]);
    match curve[0][compidx] < curve[curve.len() - 1][compidx] {
        true => curve1.iter_mut(),
        false => curve.iter_mut(),
    }
    .for_each(|p| p[compidx] -= u1 - u0);
    curve1.append(curve);
    *curve = curve1;
}

pub(super) fn periodic_axis_full_span(
    curve: &[SurfacePoint],
    surface: &impl PreMeshableSurface,
) -> Option<(usize, f64)> {
    let closed = curve
        .first()
        .zip(curve.last())
        .is_some_and(|(front, back)| front.uv.near(&back.uv));
    if curve.len() < 4 || !closed {
        None
    } else {
        [(0usize, surface.period_u()), (1usize, surface.period_v())]
            .into_iter()
            .filter_map(|(axis, period)| Some((axis, period?)))
            .find(|(axis, period)| {
                let other_axis = 1 - *axis;
                match (
                    curve_axis_span(curve, *axis),
                    curve_axis_span(curve, other_axis),
                ) {
                    (Some((min, max)), Some((other_min, other_max))) => {
                        max - min + TOLERANCE >= period * 0.75
                            && other_max - other_min <= PERIODIC_LOOP_OTHER_AXIS_TOLERANCE
                    }
                    _ => false,
                }
            })
    }
}

fn curve_axis_span(curve: &[SurfacePoint], axis: usize) -> Option<(f64, f64)> {
    curve
        .iter()
        .map(|point| point[axis])
        .fold(None, |span, value| {
            Some(match span {
                Some((min, max)) => (f64::min(min, value), f64::max(max, value)),
                None => (value, value),
            })
        })
}

fn surface_axis_range(
    surface: &impl PreMeshableSurface,
    axis: usize,
) -> (Option<f64>, Option<(f64, f64)>) {
    let (urange, vrange) = surface.try_range_tuple();
    match axis {
        0 => (surface.period_u(), urange),
        _ => (surface.period_v(), vrange),
    }
}

fn normalized_periodic_candidate(
    curve: &[SurfacePoint],
    surface: &impl PreMeshableSurface,
    axis: usize,
    period: f64,
    range: Option<(f64, f64)>,
    start: usize,
    closed_len: usize,
) -> Option<(f64, Vec<SurfacePoint>)> {
    let mut previous = None;
    let candidate = curve
        .iter()
        .take(closed_len)
        .cycle()
        .skip(start)
        .take(closed_len)
        .copied()
        .map(|mut point| {
            point[axis] =
                PolyBoundaryPiece::normalize_axis(point[axis], previous, Some(period), range)?;
            previous = Some(point[axis]);
            point.point = surface.evaluate(point.x, point.y);
            Some(point)
        })
        .collect::<Option<Vec<_>>>()?;
    let span = curve_axis_span(&candidate, axis).map(|(min, max)| max - min)?;
    Some((span, candidate))
}

pub(super) fn compact_periodic_closed_loop(
    curve: &mut Vec<SurfacePoint>,
    surface: &impl PreMeshableSurface,
) {
    let closed = curve
        .first()
        .zip(curve.last())
        .is_some_and(|(front, back)| front.uv.near(&back.uv));
    if curve.len() < 4 || !closed {
        return;
    }
    if periodic_axis_full_span(curve, surface).is_some() {
        return;
    }
    for axis in 0..=1 {
        let (Some(period), range) = surface_axis_range(surface, axis) else {
            continue;
        };
        let Some((min, max)) = curve_axis_span(curve, axis) else {
            continue;
        };
        let has_periodic_jump = curve
            .windows(2)
            .any(|points| (points[1][axis] - points[0][axis]).abs() > period * 0.5);
        if max - min <= period + TOLERANCE && !has_periodic_jump {
            continue;
        }
        let closed_len = curve.len() - 1;
        let best = (0..closed_len)
            .filter_map(|start| {
                normalized_periodic_candidate(
                    curve, surface, axis, period, range, start, closed_len,
                )
            })
            .min_by(|lhs, rhs| lhs.0.total_cmp(&rhs.0));
        if let Some((span, mut candidate)) = best
            && span <= period + TOLERANCE
        {
            candidate.push(candidate[0]);
            *curve = candidate;
        }
    }
}

fn unwrap_periodic_open_curve(curve: &mut [SurfacePoint], axis: usize, period: f64) {
    let mut previous = None;
    curve.iter_mut().for_each(|point| {
        if let Some(value) =
            PolyBoundaryPiece::normalize_axis(point[axis], previous, Some(period), None)
        {
            point[axis] = value;
        }
        previous = Some(point[axis]);
    });
}

fn shift_curve_axis(curve: &mut [SurfacePoint], axis: usize, shift: f64) {
    curve.iter_mut().for_each(|point| {
        point[axis] += shift;
    });
}

fn paired_curve_axis_span(
    curve0: &[SurfacePoint],
    curve1: &[SurfacePoint],
    axis: usize,
    curve1_shift: f64,
) -> Option<f64> {
    curve0
        .iter()
        .map(|point| point[axis])
        .chain(curve1.iter().map(|point| point[axis] + curve1_shift))
        .fold(None, |span, value| {
            Some(match span {
                Some((min, max)) => (f64::min(min, value), f64::max(max, value)),
                None => (value, value),
            })
        })
        .map(|(min, max)| max - min)
}

pub(super) fn align_periodic_open_pair(
    curve0: &[SurfacePoint],
    curve1: &mut [SurfacePoint],
    axis: usize,
    period: f64,
) {
    let best_shift = (-4..=4)
        .filter_map(|lap| {
            let shift = lap as f64 * period;
            paired_curve_axis_span(curve0, curve1, axis, shift).map(|span| (span, shift))
        })
        .min_by(|lhs, rhs| lhs.0.total_cmp(&rhs.0))
        .map(|(_, shift)| shift);
    if let Some(shift) = best_shift {
        shift_curve_axis(curve1, axis, shift);
    }
}

fn periodic_seam_bounds(
    lower: f64,
    upper: f64,
    period: f64,
    range: Option<(f64, f64)>,
) -> (f64, f64) {
    let origin = range.map(|(min, _)| min).unwrap_or(0.0);
    let mut seam_lower = origin + ((lower - origin) / period).floor() * period;
    while upper > seam_lower + period + TOLERANCE {
        seam_lower += period;
    }
    (seam_lower, seam_lower + period)
}

fn seam_surface_point(
    surface: &impl PreMeshableSurface,
    mut template: SurfacePoint,
    axis: usize,
    value: f64,
) -> SurfacePoint {
    template[axis] = value;
    template.point = surface.evaluate(template.x, template.y);
    template
}

pub(super) fn open_periodic_closed_loop(
    mut curve: Vec<SurfacePoint>,
    surface: &impl PreMeshableSurface,
    axis: usize,
    period: f64,
) -> Vec<SurfacePoint> {
    curve.pop();
    let jump = curve
        .windows(2)
        .enumerate()
        .map(|(index, points)| (index, points[1][axis] - points[0][axis]))
        .filter(|(_, delta)| delta.abs() > period * 0.5)
        .max_by(|lhs, rhs| lhs.1.abs().total_cmp(&rhs.1.abs()));
    if let Some((index, delta)) = jump {
        let lower = f64::min(curve[index][axis], curve[index + 1][axis]);
        let upper = f64::max(curve[index][axis], curve[index + 1][axis]);
        let (_, range) = surface_axis_range(surface, axis);
        let (seam_lower, seam_upper) = periodic_seam_bounds(lower, upper, period, range);
        let mut opened = Vec::with_capacity(curve.len() + 2);
        if delta > 0.0 {
            opened.push(seam_surface_point(
                surface,
                curve[index + 1],
                axis,
                seam_upper,
            ));
            opened.extend_from_slice(&curve[index + 1..]);
            opened.extend_from_slice(&curve[..=index]);
            opened.push(seam_surface_point(surface, curve[index], axis, seam_lower));
        } else {
            opened.push(seam_surface_point(
                surface,
                curve[index + 1],
                axis,
                seam_lower,
            ));
            opened.extend_from_slice(&curve[index + 1..]);
            opened.extend_from_slice(&curve[..=index]);
            opened.push(seam_surface_point(surface, curve[index], axis, seam_upper));
        }
        opened
    } else {
        unwrap_periodic_open_curve(&mut curve, axis, period);
        if let (Some(front), Some(back)) = (curve.first().copied(), curve.last().copied()) {
            let delta = back[axis] - front[axis];
            if delta.abs() + TOLERANCE < period {
                let mut seam = front;
                seam[axis] += if delta < 0.0 { -period } else { period };
                seam.point = surface.evaluate(seam.x, seam.y);
                curve.push(seam);
            }
        }
        curve
    }
}

pub(super) fn periodic_open_axis_full_span(
    curve: &[SurfacePoint],
    surface: &impl PreMeshableSurface,
) -> Option<(usize, f64)> {
    if curve.len() < 2 {
        None
    } else {
        [(0usize, surface.period_u()), (1usize, surface.period_v())]
            .into_iter()
            .filter_map(|(axis, period)| Some((axis, period?)))
            .find(|(axis, period)| {
                let other_axis = 1 - *axis;
                match (
                    curve_axis_span(curve, *axis),
                    curve_axis_span(curve, other_axis),
                ) {
                    (Some((min, max)), Some((other_min, other_max))) => {
                        max - min + TOLERANCE >= period * 0.75 && other_max - other_min <= 1.0e-6
                    }
                    _ => false,
                }
            })
    }
}

fn full_period_curve_direction(curve: &[SurfacePoint], axis: usize) -> Option<f64> {
    curve
        .first()
        .zip(curve.last())
        .map(|(front, back)| if back[axis] < front[axis] { -1.0 } else { 1.0 })
}

fn periodic_axis_value(value: f64, target_start: f64, period: f64, direction: f64) -> f64 {
    if direction < 0.0 {
        target_start - (target_start - value).rem_euclid(period)
    } else {
        target_start + (value - target_start).rem_euclid(period)
    }
}

fn periodic_seam_point(
    surface: &impl PreMeshableSurface,
    template: SurfacePoint,
    axis: usize,
    axis_value: f64,
) -> SurfacePoint {
    let mut point = template;
    point[axis] = axis_value;
    point.point = surface.evaluate(point.x, point.y);
    point
}

fn align_full_period_curve_to_axis(
    curve: &mut Vec<SurfacePoint>,
    surface: &impl PreMeshableSurface,
    axis: usize,
    period: f64,
    target_start: f64,
    direction: f64,
) -> bool {
    if curve.len() < 2 {
        false
    } else {
        if full_period_curve_direction(curve, axis).is_some_and(|current| current != direction) {
            curve.reverse();
        }
        let Some(template) = curve.first().copied() else {
            return false;
        };
        let target_end = target_start + direction * period;
        let open_len = curve.len().saturating_sub(1);
        let mut aligned = curve
            .iter()
            .take(open_len)
            .copied()
            .map(|mut point| {
                point[axis] = periodic_axis_value(point[axis], target_start, period, direction);
                point
            })
            .collect::<Vec<_>>();
        aligned.sort_by(|lhs, rhs| {
            if direction < 0.0 {
                rhs[axis].total_cmp(&lhs[axis])
            } else {
                lhs[axis].total_cmp(&rhs[axis])
            }
        });
        let mut with_seams = Vec::with_capacity(aligned.len() + 2);
        with_seams.push(periodic_seam_point(surface, template, axis, target_start));
        aligned
            .into_iter()
            .filter(|point| {
                (point[axis] - target_start).abs() > TOLERANCE
                    && (point[axis] - target_end).abs() > TOLERANCE
            })
            .for_each(|point| with_seams.push(point));
        with_seams.push(periodic_seam_point(surface, template, axis, target_end));
        *curve = with_seams;
        true
    }
}

pub(super) fn align_full_period_open_pair(
    curve0: &mut Vec<SurfacePoint>,
    curve1: &mut Vec<SurfacePoint>,
    surface: &impl PreMeshableSurface,
) -> bool {
    let Some((axis0, period0)) = periodic_open_axis_full_span(curve0, surface) else {
        return false;
    };
    let Some((axis1, period1)) = periodic_open_axis_full_span(curve1, surface) else {
        return false;
    };
    if axis0 != axis1 || (period0 - period1).abs() > TOLERANCE {
        false
    } else {
        let Some(direction) = full_period_curve_direction(curve0, axis0) else {
            return false;
        };
        let Some(start0) = curve0.first().map(|point| point[axis0]) else {
            return false;
        };
        let aligned0 =
            align_full_period_curve_to_axis(curve0, surface, axis0, period0, start0, direction);
        let Some(start1) = curve0.last().map(|point| point[axis0]) else {
            return false;
        };
        let aligned1 =
            align_full_period_curve_to_axis(curve1, surface, axis0, period0, start1, -direction);
        aligned0 && aligned1
    }
}

fn axis_derivative_norm(surface: &impl PreMeshableSurface, axis: usize, uv: Point2) -> Option<f64> {
    let norm = match axis {
        0 => surface.derivative_u(uv.x, uv.y).magnitude(),
        _ => surface.derivative_v(uv.x, uv.y).magnitude(),
    };
    norm.is_finite().then_some(norm)
}

fn uv_with_axis_values(
    template: SurfacePoint,
    axis: usize,
    axis_value: f64,
    other_value: f64,
) -> Point2 {
    let mut uv = template.uv;
    uv[axis] = axis_value;
    uv[1 - axis] = other_value;
    uv
}

fn singular_periodic_other_parameter(
    curve: &[SurfacePoint],
    surface: &impl PreMeshableSurface,
    periodic_axis: usize,
) -> Option<f64> {
    let other_axis = 1 - periodic_axis;
    let other = curve.iter().map(|point| point[other_axis]).sum::<f64>() / curve.len() as f64;
    let periodic = curve[0][periodic_axis];
    let uv = uv_with_axis_values(curve[0], periodic_axis, periodic, other);
    let radius = axis_derivative_norm(surface, periodic_axis, uv)?;
    if radius <= TOLERANCE {
        Some(other)
    } else {
        let (_, range) = surface_axis_range(surface, other_axis);
        let step = range
            .map(|(min, max)| (max - min).abs())
            .filter(|span| span.is_finite() && *span > TOLERANCE)
            .unwrap_or(DEFAULT_SINGULAR_PARAMETER_PROBE);
        [-step, step]
            .into_iter()
            .filter_map(|delta| {
                let sample_uv =
                    uv_with_axis_values(curve[0], periodic_axis, periodic, other + delta);
                let sample_radius = axis_derivative_norm(surface, periodic_axis, sample_uv)?;
                let slope = (sample_radius - radius) / delta;
                if slope.abs() <= TOLERANCE {
                    None
                } else {
                    let candidate = other - radius / slope;
                    let candidate_uv =
                        uv_with_axis_values(curve[0], periodic_axis, periodic, candidate);
                    let candidate_radius =
                        axis_derivative_norm(surface, periodic_axis, candidate_uv)?;
                    (candidate.is_finite()
                        && candidate_radius
                            <= TOLERANCE.max(radius * SINGULAR_RADIUS_RELATIVE_TOLERANCE))
                    .then_some((candidate_radius, (candidate - other).abs(), candidate))
                }
            })
            .min_by(|lhs, rhs| lhs.0.total_cmp(&rhs.0).then(lhs.1.total_cmp(&rhs.1)))
            .map(|(_, _, candidate)| candidate)
    }
}

fn singular_surface_point(
    surface: &impl PreMeshableSurface,
    mut template: SurfacePoint,
    axis: usize,
    other_value: f64,
) -> SurfacePoint {
    let other_axis = 1 - axis;
    template[other_axis] = other_value;
    template.point = surface.evaluate(template.x, template.y);
    template
}

pub(super) fn close_open_periodic_curve_to_singular(
    curve: Vec<SurfacePoint>,
    surface: &impl PreMeshableSurface,
    axis: usize,
    tolerance: f64,
    point_cache: &mut HashMap<UvKey, Point3>,
) -> Option<Vec<SurfacePoint>> {
    let singular_other = singular_periodic_other_parameter(&curve, surface, axis)?;
    let (p, q) = (*curve.first()?, *curve.last()?);
    let singular_p = singular_surface_point(surface, p, axis, singular_other);
    let singular_q = singular_surface_point(surface, q, axis, singular_other);
    let vec0 = polyline_on_surface(surface, q, singular_q, tolerance, point_cache);
    let vec1 = polyline_on_surface(surface, singular_q, singular_p, tolerance, point_cache);
    let vec2 = polyline_on_surface(surface, singular_p, p, tolerance, point_cache);
    Some(connect_edges([curve, vec0, vec1, vec2]))
}
