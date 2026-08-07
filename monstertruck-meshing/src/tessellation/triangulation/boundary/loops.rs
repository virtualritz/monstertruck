//! `PolyBoundary`: assembling projected pieces into closed, consistently
//! wound parameter-space loops, testing point inclusion, cutting isoparametric
//! curves, and feeding the constrained Delaunay triangulation.
//!
//! Split out of the module file so the source stays readable; the module path
//! is unchanged.

use super::*;
use mesh::spade_round;
use std::{f64::consts::TAU, iter, mem};

const ISOPARAM_BOUNDARY_SEARCH_ITERATIONS: usize = 32;

#[derive(Debug, Clone)]
pub(in crate::tessellation::triangulation) struct PolyBoundary {
    pub(in crate::tessellation::triangulation) loops: Vec<Vec<SurfacePoint>>,
    /// UV-space axis-aligned bounding box for cheap rejection in `include()`.
    pub(in crate::tessellation::triangulation) uv_min: Point2,
    pub(in crate::tessellation::triangulation) uv_max: Point2,
}

impl Default for PolyBoundary {
    fn default() -> Self {
        Self {
            loops: Vec::new(),
            uv_min: Point2::new(f64::INFINITY, f64::INFINITY),
            uv_max: Point2::new(f64::NEG_INFINITY, f64::NEG_INFINITY),
        }
    }
}

fn loop_signed_area(curve: &[SurfacePoint]) -> f64 {
    curve
        .iter()
        .circular_tuple_windows()
        .fold(0.0, |sum, (p, q)| sum + (q.x + p.x) * (q.y - p.y))
}

fn loop_orientation(curve: &[SurfacePoint]) -> bool { loop_signed_area(curve) > 0.0 }

fn normalize_closed_loop_winding(closed: &mut [Vec<SurfacePoint>]) {
    let outer_index = closed
        .iter()
        .enumerate()
        .max_by(|(_, lhs), (_, rhs)| {
            loop_signed_area(lhs)
                .abs()
                .total_cmp(&loop_signed_area(rhs).abs())
        })
        .map(|(index, _)| index);
    if let Some(outer_index) = outer_index {
        closed.iter_mut().enumerate().for_each(|(index, curve)| {
            let should_be_positive = index == outer_index;
            if loop_orientation(curve) != should_be_positive {
                curve.reverse();
            }
        });
    }
}

fn open_pair_connector_score(
    p0: SurfacePoint,
    p1: SurfacePoint,
    q0: SurfacePoint,
    q1: SurfacePoint,
) -> f64 {
    let p1q0 = p1.uv - q0.uv;
    let q1p0 = q1.uv - p0.uv;
    p1q0.x * p1q0.x + p1q0.y * p1q0.y + q1p0.x * q1p0.x + q1p0.y * q1p0.y
}

pub(in crate::tessellation::triangulation) fn boundary_segment_parameter(
    point: Point2,
    front: Point2,
    back: Point2,
) -> Option<f64> {
    let segment = back - front;
    let denom = segment.dot(segment);
    if denom.so_small() {
        None
    } else {
        let offset = point - front;
        let parameter = offset.dot(segment) / denom;
        let projected = front + segment * parameter;
        (parameter > 0.0 && parameter < 1.0 && projected.distance(point) <= 1.0e-9)
            .then_some(parameter)
    }
}

fn push_isoparam_uv(curve: &mut Vec<Point2>, uv: Point2) {
    if curve.last().is_none_or(|previous| !previous.near(&uv)) {
        curve.push(uv);
    }
}

fn push_finished_isoparam_curve(curves: &mut Vec<Vec<Point2>>, curve: &mut Vec<Point2>) {
    if curve.len() >= 2 {
        curves.push(mem::take(curve));
    } else {
        curve.clear();
    }
}

fn add_direct_cdt_constraint(
    triangulation: &mut Cdt,
    added_constraints: &mut usize,
    skipped_constraints: &mut usize,
    front: FixedVertexHandle,
    back: FixedVertexHandle,
) -> bool {
    if front == back || !triangulation.can_add_constraint(front, back) {
        *skipped_constraints += 1;
        false
    } else {
        let constraints = triangulation.add_constraint_and_split(front, back, |point| point);
        if constraints.is_empty() {
            *skipped_constraints += 1;
            false
        } else {
            *added_constraints += constraints.len();
            true
        }
    }
}

impl PolyBoundary {
    pub(in crate::tessellation::triangulation) fn new(
        pieces: Vec<PolyBoundaryPiece>,
        surface: &impl PreMeshableSurface,
        tolerance: f64,
    ) -> Self {
        let (mut closed, mut open) = (Vec::new(), Vec::new());
        pieces.into_iter().for_each(|PolyBoundaryPiece(mut vec)| {
            compact_periodic_closed_loop(&mut vec, surface);
            if let Some((axis, period)) = periodic_axis_full_span(&vec, surface) {
                open.push(open_periodic_closed_loop(vec, surface, axis, period));
            } else {
                match vec[0].uv.distance(vec[vec.len() - 1].uv) < 1.0e-3 {
                    true => {
                        vec.pop();
                        closed.push(vec)
                    }
                    false => open.push(vec),
                }
            }
        });
        open.retain(|curve| curve.len() >= 2);
        closed.retain(|curve| curve.len() >= 2);
        let mut point_cache = HashMap::<UvKey, Point3>::default();
        match open.len() {
            1 => {
                // SAFETY: open.len() == 1 was matched above.
                let mut curve = open.pop().unwrap();
                if let Some((axis, _)) = periodic_open_axis_full_span(&curve, surface)
                    && let Some(singular_curve) = close_open_periodic_curve_to_singular(
                        curve.clone(),
                        surface,
                        axis,
                        tolerance,
                        &mut point_cache,
                    )
                {
                    closed.push(singular_curve);
                } else if let (Some((u0, u1)), Some((v0, v1))) = surface.try_range_tuple() {
                    let p = curve[0];
                    let q = curve[curve.len() - 1];
                    if p.x < q.x - TOLERANCE {
                        normalize_range(&mut curve, 0, (u0, u1));
                        let p = curve[0];
                        let q = curve[curve.len() - 1];
                        let x = surface_point_with_cache(
                            surface,
                            Point2::new(u0, v1),
                            &mut point_cache,
                        );
                        let y = surface_point_with_cache(
                            surface,
                            Point2::new(u1, v1),
                            &mut point_cache,
                        );
                        let vec0 = polyline_on_surface(surface, q, y, tolerance, &mut point_cache);
                        let vec1 = polyline_on_surface(surface, y, x, tolerance, &mut point_cache);
                        let vec2 = polyline_on_surface(surface, x, p, tolerance, &mut point_cache);
                        closed.push(connect_edges([vec0, vec1, vec2, curve]));
                    } else if q.x < p.x - TOLERANCE {
                        normalize_range(&mut curve, 0, (u0, u1));
                        let p = curve[0];
                        let q = curve[curve.len() - 1];
                        let x = surface_point_with_cache(
                            surface,
                            Point2::new(u1, v0),
                            &mut point_cache,
                        );
                        let y = surface_point_with_cache(
                            surface,
                            Point2::new(u0, v0),
                            &mut point_cache,
                        );
                        let vec0 = polyline_on_surface(surface, q, y, tolerance, &mut point_cache);
                        let vec1 = polyline_on_surface(surface, y, x, tolerance, &mut point_cache);
                        let vec2 = polyline_on_surface(surface, x, p, tolerance, &mut point_cache);
                        closed.push(connect_edges([vec0, vec1, vec2, curve]));
                    } else if p.y < q.y - TOLERANCE {
                        normalize_range(&mut curve, 1, (v0, v1));
                        let p = curve[0];
                        let q = curve[curve.len() - 1];
                        let x = surface_point_with_cache(
                            surface,
                            Point2::new(u0, v0),
                            &mut point_cache,
                        );
                        let y = surface_point_with_cache(
                            surface,
                            Point2::new(u0, v1),
                            &mut point_cache,
                        );
                        let vec0 = polyline_on_surface(surface, q, y, tolerance, &mut point_cache);
                        let vec1 = polyline_on_surface(surface, y, x, tolerance, &mut point_cache);
                        let vec2 = polyline_on_surface(surface, x, p, tolerance, &mut point_cache);
                        closed.push(connect_edges([vec0, vec1, vec2, curve]));
                    } else if q.y < p.y - TOLERANCE {
                        normalize_range(&mut curve, 1, (v0, v1));
                        let p = curve[0];
                        let q = curve[curve.len() - 1];
                        let x = surface_point_with_cache(
                            surface,
                            Point2::new(u1, v1),
                            &mut point_cache,
                        );
                        let y = surface_point_with_cache(
                            surface,
                            Point2::new(u1, v0),
                            &mut point_cache,
                        );
                        let vec0 = polyline_on_surface(surface, q, y, tolerance, &mut point_cache);
                        let vec1 = polyline_on_surface(surface, y, x, tolerance, &mut point_cache);
                        let vec2 = polyline_on_surface(surface, x, p, tolerance, &mut point_cache);
                        closed.push(connect_edges([vec0, vec1, vec2, curve]));
                    }
                }
            }
            2 => {
                // SAFETY: open.len() == 2 was matched above.
                let mut curve1 = open.pop().unwrap();
                let mut curve0 = open.pop().unwrap();
                fn end_pts<T: Copy>(vec: &[T]) -> (T, T) { (vec[0], vec[vec.len() - 1]) }
                let full_period_pair_aligned =
                    align_full_period_open_pair(&mut curve0, &mut curve1, surface);
                let ((p0, p1), (q0, q1)) = (end_pts(&curve0), end_pts(&curve1));
                if !full_period_pair_aligned && !p0.x.near(&p1.x) && !q0.x.near(&q1.x) {
                    if let Some(period) = surface.period_u() {
                        align_periodic_open_pair(&curve0, &mut curve1, 0, period);
                    } else if let (Some(urange), _) = surface.try_range_tuple() {
                        normalize_range(&mut curve0, 0, urange);
                        normalize_range(&mut curve1, 0, urange);
                    }
                } else if !full_period_pair_aligned && !p0.y.near(&p1.y) && !q0.y.near(&q1.y) {
                    if let Some(period) = surface.period_v() {
                        align_periodic_open_pair(&curve0, &mut curve1, 1, period);
                    } else if let (_, Some(vrange)) = surface.try_range_tuple() {
                        normalize_range(&mut curve0, 1, vrange);
                        normalize_range(&mut curve1, 1, vrange);
                    }
                }
                let ((p0, p1), (q0, q1)) = (end_pts(&curve0), end_pts(&curve1));
                let current_score = open_pair_connector_score(p0, p1, q0, q1);
                let reversed_score = open_pair_connector_score(p0, p1, q1, q0);
                if reversed_score + TOLERANCE < current_score {
                    curve1.reverse();
                }
                let ((p0, p1), (q0, q1)) = (end_pts(&curve0), end_pts(&curve1));
                let vec0 = polyline_on_surface(surface, p1, q0, tolerance, &mut point_cache);
                let vec1 = polyline_on_surface(surface, q1, p0, tolerance, &mut point_cache);
                closed.push(connect_edges([curve0, vec0, curve1, vec1]));
            }
            _ => {}
        }
        if closed.len() == 1 && !loop_orientation(&closed[0]) {
            closed[0].reverse();
        }
        if !closed.iter().any(|curve| loop_orientation(curve))
            && let (Some((u0, u1)), Some((v0, v1))) = surface.try_range_tuple()
        {
            let p = [
                surface_point_with_cache(surface, Point2::new(u0, v0), &mut point_cache),
                surface_point_with_cache(surface, Point2::new(u1, v0), &mut point_cache),
                surface_point_with_cache(surface, Point2::new(u1, v1), &mut point_cache),
                surface_point_with_cache(surface, Point2::new(u0, v1), &mut point_cache),
            ];
            let vec0 = polyline_on_surface(surface, p[0], p[1], tolerance, &mut point_cache);
            let vec1 = polyline_on_surface(surface, p[1], p[2], tolerance, &mut point_cache);
            let vec2 = polyline_on_surface(surface, p[2], p[3], tolerance, &mut point_cache);
            let vec3 = polyline_on_surface(surface, p[3], p[0], tolerance, &mut point_cache);
            closed.push(connect_edges([vec0, vec1, vec2, vec3]));
        }
        normalize_closed_loop_winding(&mut closed);
        let (mut uv_min, mut uv_max) = (
            Point2::new(f64::INFINITY, f64::INFINITY),
            Point2::new(f64::NEG_INFINITY, f64::NEG_INFINITY),
        );
        for pt in closed.iter().flatten() {
            uv_min.x = f64::min(uv_min.x, pt.x);
            uv_min.y = f64::min(uv_min.y, pt.y);
            uv_max.x = f64::max(uv_max.x, pt.x);
            uv_max.y = f64::max(uv_max.y, pt.y);
        }
        Self {
            loops: closed,
            uv_min,
            uv_max,
        }
    }

    /// Whether `c` is included in the domain with boundary = `self`.
    pub(in crate::tessellation::triangulation) fn include(&self, c: Point2) -> bool {
        // AABB early reject.
        if c.x < self.uv_min.x || c.x > self.uv_max.x || c.y < self.uv_min.y || c.y > self.uv_max.y
        {
            return false;
        }
        let t = TAU * HashGen::hash1(c);
        let r = Vector2::new(f64::cos(t), f64::sin(t));
        self.loops
            .iter()
            .flat_map(|vec| vec.iter().circular_tuple_windows())
            .try_fold(0_i32, move |counter, (p0, p1)| {
                let a = **p0 - c;
                let b = **p1 - c;
                let s0 = r.x * a.y - r.y * a.x; // v times a.
                let s1 = r.x * b.y - r.y * b.x; // v times b.
                let s2 = a.x * b.y - a.y * b.x; // a times b.
                let x = s2 / (s1 - s0);
                if x.so_small() && s0 * s1 < 0.0 {
                    None
                } else if x > 0.0 && s0 <= 0.0 && s1 > 0.0 {
                    Some(counter + 1)
                } else if x > 0.0 && s0 >= 0.0 && s1 < 0.0 {
                    Some(counter - 1)
                } else {
                    Some(counter)
                }
            })
            .map(|counter| counter > 0)
            .unwrap_or(false)
    }

    pub(in crate::tessellation::triangulation) fn isoparametric_curves<S: PreMeshableSurface>(
        &self,
        surface: &S,
        options: IsoparametricCurveOptions,
    ) -> Vec<Vec<Point3>> {
        if self.loops.is_empty()
            || options.samples_per_direction == 0
            || options.segments_per_curve == 0
            || self.uv_min.x >= self.uv_max.x
            || self.uv_min.y >= self.uv_max.y
        {
            Vec::new()
        } else {
            (0..=1)
                .flat_map(|axis| {
                    self.isoparametric_axis_values(axis, options.samples_per_direction)
                        .into_iter()
                        .flat_map(move |axis_value| {
                            self.isoparametric_uv_curves(
                                axis,
                                axis_value,
                                options.segments_per_curve,
                            )
                            .into_iter()
                            .map(|curve| {
                                curve
                                    .into_iter()
                                    .map(|uv| surface.evaluate(uv.x, uv.y))
                                    .collect::<Vec<_>>()
                            })
                        })
                })
                .collect()
        }
    }

    fn isoparametric_axis_values(&self, axis: usize, count: usize) -> Vec<f64> {
        let (min, max) = match axis {
            0 => (self.uv_min.x, self.uv_max.x),
            _ => (self.uv_min.y, self.uv_max.y),
        };
        (0..count)
            .map(|index| {
                let parameter = (index + 1) as f64 / (count + 1) as f64;
                min + (max - min) * parameter
            })
            .collect()
    }

    fn isoparametric_uv_curves(
        &self,
        constant_axis: usize,
        constant_value: f64,
        segments: usize,
    ) -> Vec<Vec<Point2>> {
        let variable_axis = 1 - constant_axis;
        let (min, max) = match variable_axis {
            0 => (self.uv_min.x, self.uv_max.x),
            _ => (self.uv_min.y, self.uv_max.y),
        };
        let samples = (0..=segments)
            .map(|index| {
                let parameter = index as f64 / segments as f64;
                let variable_value = min + (max - min) * parameter;
                match constant_axis {
                    0 => Point2::new(constant_value, variable_value),
                    _ => Point2::new(variable_value, constant_value),
                }
            })
            .collect::<Vec<_>>();
        self.trim_uv_samples(samples)
    }

    fn trim_uv_samples(&self, samples: Vec<Point2>) -> Vec<Vec<Point2>> {
        let (_, mut curves, mut current) = samples.into_iter().fold(
            (
                None::<(Point2, bool)>,
                Vec::<Vec<Point2>>::new(),
                Vec::<Point2>::new(),
            ),
            |(previous, mut curves, mut current), uv| {
                let included = self.include(uv);
                if let Some((previous_uv, previous_included)) = previous
                    && previous_included != included
                {
                    let crossing =
                        self.isoparametric_boundary_crossing(previous_uv, uv, previous_included);
                    push_isoparam_uv(&mut current, crossing);
                    if previous_included {
                        push_finished_isoparam_curve(&mut curves, &mut current);
                    }
                    if included {
                        push_isoparam_uv(&mut current, crossing);
                    }
                }
                if included {
                    push_isoparam_uv(&mut current, uv);
                }
                (Some((uv, included)), curves, current)
            },
        );
        push_finished_isoparam_curve(&mut curves, &mut current);
        curves
    }

    fn isoparametric_boundary_crossing(
        &self,
        front: Point2,
        back: Point2,
        front_included: bool,
    ) -> Point2 {
        let (mut included, mut excluded) = match front_included {
            true => (front, back),
            false => (back, front),
        };
        (0..ISOPARAM_BOUNDARY_SEARCH_ITERATIONS).for_each(|_| {
            let middle = Point2::new(
                (included.x + excluded.x) * 0.5,
                (included.y + excluded.y) * 0.5,
            );
            if self.include(middle) {
                included = middle;
            } else {
                excluded = middle;
            }
        });
        Point2::new(
            (included.x + excluded.x) * 0.5,
            (included.y + excluded.y) * 0.5,
        )
    }

    /// Inserts points and adds constraint into triangulation.
    pub(in crate::tessellation::triangulation) fn insert_to(
        &self,
        triangulation: &mut Cdt,
        boundary_map: &mut HashMap<FixedVertexHandle, Point3>,
    ) -> (usize, usize, usize) {
        let poly2tri: Vec<_> = self
            .loops
            .iter()
            .flatten()
            .map(|pt| {
                let p = [spade_round(pt.x), spade_round(pt.y)];
                match triangulation.insert(SPoint2::from(p)) {
                    Err(_) => None,
                    Ok(idx) => {
                        boundary_map.insert(idx, pt.point);
                        Some(idx)
                    }
                }
            })
            .collect();
        let split_vertices = triangulation
            .vertices()
            .map(|vertex| {
                let point = vertex.position();
                (vertex.fix(), Point2::new(point.x, point.y))
            })
            .collect::<Vec<_>>();
        let mut counter = 0;
        let mut added_constraints = 0usize;
        let mut skipped_constraints = 0usize;
        let mut add_constraint = |front: FixedVertexHandle, back: FixedVertexHandle| {
            if front == back {
                false
            } else {
                let front_point = triangulation.vertex(front).position();
                let back_point = triangulation.vertex(back).position();
                let front_point = Point2::new(front_point.x, front_point.y);
                let back_point = Point2::new(back_point.x, back_point.y);
                let mut chain = split_vertices
                    .iter()
                    .filter_map(|(handle, point)| {
                        if *handle == front || *handle == back {
                            None
                        } else {
                            boundary_segment_parameter(*point, front_point, back_point)
                                .map(|parameter| (parameter, *handle))
                        }
                    })
                    .collect::<Vec<_>>();
                chain.sort_by(|lhs, rhs| lhs.0.total_cmp(&rhs.0));
                let handles = iter::once(front)
                    .chain(chain.into_iter().map(|(_, handle)| handle))
                    .chain(iter::once(back))
                    .collect::<Vec<_>>();
                let mut handled = false;
                handles.windows(2).for_each(|window| {
                    if window[0] != window[1] {
                        handled = true;
                        add_direct_cdt_constraint(
                            triangulation,
                            &mut added_constraints,
                            &mut skipped_constraints,
                            window[0],
                            window[1],
                        );
                    }
                });
                handled
            }
        };
        self.loops.iter().map(Vec::len).for_each(|len| {
            let range = counter..counter + len;
            counter += len;
            let mut prev: Option<usize> = None;
            range.circular_tuple_windows().for_each(|(i, j)| {
                let Some(vj) = poly2tri[j] else { return };
                if let Some(p) = prev {
                    let Some(v) = poly2tri[p] else { return };
                    if add_constraint(v, vj) {
                        prev = None;
                    }
                } else {
                    let Some(vi) = poly2tri[i] else { return };
                    if !add_constraint(vi, vj) {
                        prev = Some(i);
                    }
                }
            });
        });
        (boundary_map.len(), added_constraints, skipped_constraints)
    }
}
