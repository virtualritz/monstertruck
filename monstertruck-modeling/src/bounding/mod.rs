//! Axis-aligned bounds that are actually BOUNDS.
//!
//! # Why this module exists
//!
//! The obvious way to box a B-rep solid -- fold its vertices into a
//! [`BoundingBox`] -- is not a bounding box at all once a face is curved. A
//! vertex is a CORNER of the topology; the surface between corners bulges past
//! it, and nothing in the B-rep says by how much. The measured witness (spec
//! 013, ledger class C15) is ROTOR `#25387`: a sphere of radius `12.5` sliced to
//! the slab `|x| <= 9` and bored by a cylinder of radius `6`. Its vertices all
//! sit on the four trim circles, so the vertex box is `18 x 17.349 x 12`
//! (`3747.4`), while the solid's own closed-form volume is `5273.16` -- the
//! CORRECT volume exceeds its "bounding" box by 40.7%. Any test that certifies
//! a volume with `v <= vertex_box` therefore rejects correct answers and
//! accepts a whole family of wrong ones.
//!
//! What this module computes instead is a bound with a proof attached, per
//! face, and it returns [`None`] rather than guess when it has no proof. A
//! caller that gets `None` has learned something true (this solid carries a
//! face class we cannot certify); a caller that gets `Some` has a box the solid
//! provably fits inside.
//!
//! # The per-face arguments
//!
//! A solid is contained in the convex hull of its boundary, and its boundary is
//! the union of its faces, so a box containing every face contains the solid.
//! Each surface class gets its own argument:
//!
//! * **B-spline / NURBS surface** -- the convex-hull property: a B-spline patch
//!   lies in the convex hull of its control net, and a rational patch with
//!   strictly positive weights lies in the convex hull of the PROJECTED control
//!   points. A non-positive weight voids the property, so it refuses.
//! * **Plane** -- a planar face is the region its boundary wires enclose, so it
//!   lies in the convex hull of those wires. Bound the wires (see below) and
//!   the face is bounded.
//! * **Sphere / torus** -- bound the WHOLE analytic surface, untrimmed. Sound
//!   by construction, and loose by exactly the amount the trim would have
//!   removed (see [`analytic_sphere_looseness`] for the closed form).
//! * **Surface of revolution over a straight profile** (this is what STEP
//!   cylinders and cones become: `RevolutionSurface<Line>`) -- the untrimmed
//!   surface is UNBOUNDED, since the profile line's parameter is not clamped to
//!   the segment the STEP builder handed it. But on such a surface both the
//!   axial coordinate and the distance from the axis are functions of the
//!   profile parameter `u` ALONE. The face is compact and connected, so its
//!   `u`-projection is an interval whose ends are attained on its boundary; the
//!   axial coordinate is affine in `u` and the radial distance is convex in
//!   `u`, so both attain their extremes over the face at those same ends.
//!   Bounding the face's boundary wires therefore bounds the face.
//!
//! Anything else -- a T-spline patch, a surface behind a non-similarity
//! transform, a general surface of revolution -- refuses.
//!
//! # Bounding a wire
//!
//! Edge curves get the same treatment: a segment is its two endpoints, a
//! B-spline curve its control polygon, a rational curve its projected control
//! polygon (positive weights only). A parameter curve or an intersection curve
//! carries no cheap hull -- its `leader` is an APPROXIMATION, not an enclosure
//! -- so those refuse too.
//!
//! # This is a bound, not a tight bound
//!
//! Untrimmed analytic surfaces are the loose part. On `#25387` the sphere faces
//! alone force the full `25 x 25 x 25` box (`15625`) where the solid actually
//! occupies `18 x 25 x 25`. That is sound and it is ~3x the solid's volume; a
//! caller wanting tightness should say so and get a different instrument, not a
//! quietly-unsound one. MEASURED over the four in-repo fixtures, as a ratio of
//! the certified box to the vertex hull: boxy 1.00x, ap224 1.15x, io1 2.95x,
//! coffy 71.5x. The tail is the untrimmed-surface classes, and coffy shows what
//! they cost when a face is a small trim of a large surface.
//!
//! # What it assumes, stated so nobody has to guess
//!
//! * A B-spline / NURBS patch or curve is evaluated within its own knot domain.
//!   The convex-hull property is a statement about that domain; extrapolation
//!   beyond it is not bounded by the control net and is not claimed here.
//!   (Trims are erased before geometry mapping, so a face's domain is its
//!   surface's domain.)
//! * A face is compact and connected -- used only in the revolution argument,
//!   to say that a `u`-extreme of the face is attained on its boundary.
//! * A `Processor`'s `Matrix4` is a similarity. It is checked, not assumed:
//!   anything else refuses.

use crate::*;

/// How much an analytic sphere's axis-aligned bounding box exceeds the sphere:
/// `(2r)^3 / (4/3 pi r^3) = 6 / pi`, independent of the radius.
///
/// Stated as a constant because it is the honest answer to "how loose is this
/// bound?" for the commonest curved class: a certified box around a ball is
/// 91.0% larger than the ball. Anything much looser than this on a
/// sphere-dominated solid is a bug in the caller's expectations, not in the
/// bound.
pub const ANALYTIC_SPHERE_LOOSENESS: f64 = 6.0 / std::f64::consts::PI;

/// The ratio of the certified box's volume to the enclosed ball's volume, for a
/// ball of any radius: exactly [`ANALYTIC_SPHERE_LOOSENESS`].
#[inline]
pub const fn analytic_sphere_looseness() -> f64 { ANALYTIC_SPHERE_LOOSENESS }

/// Relative tolerance for judging a `Matrix4` a similarity.
const SIMILARITY_TOL: f64 = 1.0e-9;

/// A certified axis-aligned box containing every point of `solid`, or [`None`]
/// when any one of its faces carries a class this module cannot prove a bound
/// for.
///
/// The box contains the solid's SURFACE and therefore -- the surface being a
/// closed boundary -- the solid. See the module note for the per-class
/// arguments and for how loose the result is.
pub fn certified_solid_bounding_box(solid: &Solid) -> Option<BoundingBox<Point3>> {
    let mut result = BoundingBox::<Point3>::new();
    let mut any = false;
    for shell in solid.boundaries() {
        for face in shell.face_iter() {
            result += certified_face_bounding_box(face)?;
            any = true;
        }
    }
    any.then_some(result)
}

/// A certified axis-aligned box containing every point of `face`, or [`None`].
pub fn certified_face_bounding_box(face: &Face) -> Option<BoundingBox<Point3>> {
    let surface = face.surface();
    let hull = face_boundary_hull(face);
    certified_surface_bounding_box(&surface, hull.as_deref())
}

/// A certified axis-aligned box containing a face carried by `surface` whose
/// boundary wires are enclosed by the convex hull of `boundary_hull`, or
/// [`None`] when no argument in this module applies.
///
/// `boundary_hull` may be [`None`] (the caller could not bound the wires); the
/// classes that need it then refuse, while the self-bounded analytic and
/// control-net classes still answer.
pub fn certified_surface_bounding_box(
    surface: &Surface,
    boundary_hull: Option<&[Point3]>,
) -> Option<BoundingBox<Point3>> {
    match surface {
        // A planar face is exactly what its wires enclose.
        Surface::Plane(_) => hull_box(boundary_hull?),
        // Convex-hull property, straight.
        Surface::BsplineSurface(surface) => {
            hull_box_iter(surface.control_points().iter().flatten().copied())
        }
        // Convex-hull property, rational: only with strictly positive weights.
        Surface::NurbsSurface(surface) => {
            let mut points = Vec::new();
            for row in surface.control_points() {
                for control in row {
                    points.push(projected_control_point(*control)?);
                }
            }
            hull_box(&points)
        }
        // The whole ball, untrimmed.
        Surface::SphericalSurface(processor) => {
            let scale = similarity_scale(processor.transform())?;
            let sphere = processor.entity();
            let center = transform_point(processor.transform(), sphere.center());
            let radius = scale * sphere.radius();
            (radius.is_finite() && radius >= 0.0).then(|| {
                let offset = Vector3::new(radius, radius, radius);
                BoundingBox::from_iter([center - offset, center + offset])
            })
        }
        // The whole ring torus, untrimmed: `|radial| <= R + r`, `|axial| <= r`
        // about its own axis, which the transform carries to world space.
        Surface::ToroidalSurface(processor) => {
            let scale = similarity_scale(processor.transform())?;
            let torus = processor.entity();
            let center = transform_point(processor.transform(), torus.center());
            let axis = transform_direction(processor.transform(), Vector3::unit_z())?;
            let large = scale * torus.large_radius().abs();
            let small = scale * torus.small_radius().abs();
            (large.is_finite() && small.is_finite())
                .then(|| revolution_region_box(center, axis, (-small, small), large + small))
        }
        // Cylinders and cones. Bounded only via the face's own wires -- see the
        // module note on why the untrimmed surface is not bounded at all.
        Surface::RevolutionSurface(processor) => {
            let revolution = processor.entity();
            // The convexity/affineness argument is stated for a STRAIGHT
            // profile and is claimed for nothing else.
            if !matches!(revolution.entity_curve(), Curve::Line(_)) {
                return None;
            }
            similarity_scale(processor.transform())?;
            let origin = transform_point(processor.transform(), revolution.origin());
            let axis = transform_direction(processor.transform(), revolution.axis())?;
            revolution_face_box(origin, axis, boundary_hull?)
        }
        // No hull argument claimed for a T-mesh here.
        Surface::TsplineSurface(_) => None,
    }
}

/// The face classes of `solid` that [`certified_solid_bounding_box`] declines,
/// each as `surface-class[/curve-class ...]`, deduplicated and sorted.
///
/// Diagnosis, not certification: an empty result means every face was bounded.
/// A caller staring at a `None` needs to know WHICH class it tripped over, and
/// guessing is how instruments start lying.
pub fn uncertifiable_face_classes(solid: &Solid) -> Vec<String> {
    let mut classes: Vec<String> = Vec::new();
    for shell in solid.boundaries() {
        for face in shell.face_iter() {
            if certified_face_bounding_box(face).is_some() {
                continue;
            }
            let surface = face.surface();
            let mut blockers: Vec<&'static str> = face
                .edge_iter()
                .filter_map(|edge| {
                    let curve = edge.curve();
                    let mut sink = Vec::new();
                    push_curve_hull(&curve, &mut sink)
                        .is_none()
                        .then(|| curve_class_name(&curve))
                })
                .collect();
            blockers.sort_unstable();
            blockers.dedup();
            let mut label = surface_class_name(&surface).to_string();
            for blocker in blockers {
                label.push('/');
                label.push_str(blocker);
            }
            classes.push(label);
        }
    }
    classes.sort();
    classes.dedup();
    classes
}

/// The [`Surface`] variant's name, for diagnosis.
fn surface_class_name(surface: &Surface) -> &'static str {
    match surface {
        Surface::Plane(_) => "Plane",
        Surface::BsplineSurface(_) => "BsplineSurface",
        Surface::NurbsSurface(_) => "NurbsSurface",
        Surface::RevolutionSurface(_) => "RevolutionSurface",
        Surface::TsplineSurface(_) => "TsplineSurface",
        Surface::SphericalSurface(_) => "SphericalSurface",
        Surface::ToroidalSurface(_) => "ToroidalSurface",
    }
}

/// The [`Curve`] variant's name, for diagnosis.
fn curve_class_name(curve: &Curve) -> &'static str {
    match curve {
        Curve::Line(_) => "Line",
        Curve::BsplineCurve(_) => "BsplineCurve",
        Curve::NurbsCurve(_) => "NurbsCurve",
        Curve::ParameterCurve(_) => "ParameterCurve",
        Curve::IntersectionCurve(_) => "IntersectionCurve",
    }
}

/// Points whose convex hull encloses every boundary wire of `face`, or [`None`]
/// when some edge carries a curve with no cheap enclosure.
pub fn face_boundary_hull(face: &Face) -> Option<Vec<Point3>> {
    let mut points = Vec::new();
    for edge in face.edge_iter() {
        push_curve_hull(&edge.curve(), &mut points)?;
    }
    (!points.is_empty()).then_some(points)
}

/// Appends points whose convex hull encloses `curve`, or returns [`None`] when
/// the curve class carries no such finite point set.
///
/// The two composite classes never touch their stored `leader`: a leader is a
/// FITTED approximation of the curve and an approximation is not an enclosure.
/// They are bounded structurally instead --
///
/// * a parameter curve `surface(c(t))` by pushing `c`'s 2D control hull through
///   [`surface_patch_box`], and
/// * an intersection curve by the fact that it lies on BOTH its surfaces, so
///   any sound box for EITHER of them (or for either boundary parameter curve,
///   which is tighter) contains it.
///
/// -- and the box's eight corners are pushed, a convex hull containing it.
pub fn push_curve_hull(curve: &Curve, points: &mut Vec<Point3>) -> Option<()> {
    match curve {
        Curve::Line(Line(front, back)) => {
            points.push(*front);
            points.push(*back);
            Some(())
        }
        Curve::BsplineCurve(curve) => {
            points.extend(curve.control_points().iter().copied());
            Some(())
        }
        Curve::NurbsCurve(curve) => {
            for control in curve.control_points() {
                points.push(projected_control_point(*control)?);
            }
            Some(())
        }
        Curve::ParameterCurve(pcurve) => {
            push_box_corners(&parameter_curve_box(pcurve)?, points);
            Some(())
        }
        Curve::IntersectionCurve(curve) => {
            // FOUR independent enclosures, each sound on its own, so their
            // INTERSECTION is sound and no looser than the best of them.
            let candidates = [
                curve.boundary0().and_then(parameter_curve_box),
                curve.boundary1().and_then(parameter_curve_box),
                certified_surface_bounding_box(curve.surface0(), None),
                certified_surface_bounding_box(curve.surface1(), None),
            ];
            let mut tightest: Option<BoundingBox<Point3>> = None;
            for candidate in candidates.into_iter().flatten() {
                tightest = Some(match tightest {
                    None => candidate,
                    Some(current) => intersect_boxes(current, candidate)?,
                });
            }
            push_box_corners(&tightest?, points);
            Some(())
        }
    }
}

/// A sound box for the 3D curve `surface(c(t))`.
fn parameter_curve_box(
    pcurve: &ParameterCurve<Curve2D, Box<Surface>>,
) -> Option<BoundingBox<Point3>> {
    surface_patch_box(pcurve.surface(), parameter_curve_uv_box(pcurve.curve())?)
}

/// A `(u, v)` rectangle containing every point of a 2D parameter curve.
///
/// Same convex-hull arguments as the 3D case, plus: a trimmed unit circle is an
/// arc of the unit circle, so the unit square bounds it whatever the trim, and
/// the `Matrix3` placement carries that square across affinely. A trimmed
/// hyperbola or parabola gets no bound here -- both are convex arcs whose
/// interior escapes the hull of their ends, and no cheap enclosure is claimed.
fn parameter_curve_uv_box(curve: &Curve2D) -> Option<BoundingBox<Point2>> {
    let mut points: Vec<Point2> = Vec::new();
    match curve {
        Curve2D::Line(Line(front, back)) => points.extend([*front, *back]),
        Curve2D::Polyline(polyline) => points.extend(polyline.0.iter().copied()),
        Curve2D::BsplineCurve(curve) => points.extend(curve.control_points().iter().copied()),
        Curve2D::NurbsCurve(curve) => {
            for control in curve.control_points() {
                let weight = control.z;
                if !(weight > 0.0 && weight.is_finite()) {
                    return None;
                }
                points.push(Point2::new(control.x / weight, control.y / weight));
            }
        }
        Curve2D::Conic(Conic2D::Ellipse(processor)) => {
            let matrix = processor.transform();
            for (x, y) in [(-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)] {
                let mapped = matrix * Vector3::new(x, y, 1.0);
                if (mapped.z - 1.0).abs() > 1.0e-9 {
                    return None;
                }
                points.push(Point2::new(mapped.x, mapped.y));
            }
        }
        Curve2D::Conic(Conic2D::Hyperbola(_) | Conic2D::Parabola(_)) => return None,
    }
    let mut result = BoundingBox::<Point2>::new();
    let mut any = false;
    for point in points {
        if !point.x.is_finite() || !point.y.is_finite() {
            return None;
        }
        result.push(point);
        any = true;
    }
    any.then_some(result)
}

/// A sound box for `surface` restricted to the `(u, v)` rectangle `domain`.
///
/// Only two classes exploit `domain`: a plane, where the map is affine so the
/// rectangle's four corners bound the image exactly, and a straight-profile
/// revolution, where the same axial-affine / radial-convex argument as
/// [`revolution_face_box`] applies to the `u`-interval, with a FULL turn
/// assumed in `v`. Every other class ignores `domain` and falls back to its
/// whole-surface bound, which is sound and merely loose.
fn surface_patch_box(
    surface: &Surface,
    domain: BoundingBox<Point2>,
) -> Option<BoundingBox<Point3>> {
    let (low, high) = (domain.min(), domain.max());
    match surface {
        Surface::Plane(plane) => {
            let mut points = Vec::with_capacity(4);
            for (u, v) in [
                (low.x, low.y),
                (low.x, high.y),
                (high.x, low.y),
                (high.x, high.y),
            ] {
                points.push(plane.origin() + plane.axis_u() * u + plane.axis_v() * v);
            }
            hull_box(&points)
        }
        Surface::RevolutionSurface(processor) => {
            let revolution = processor.entity();
            let Curve::Line(Line(front, back)) = revolution.entity_curve() else {
                return None;
            };
            similarity_scale(processor.transform())?;
            let matrix = processor.transform();
            let origin = transform_point(matrix, revolution.origin());
            let axis = transform_direction(matrix, revolution.axis())?;
            let (front, back) = (
                transform_point(matrix, *front),
                transform_point(matrix, *back),
            );
            // The profile is affine in `u`, so its ends over `[u_low, u_high]`
            // pin the axial interval, and the radius -- convex in `u` -- takes
            // its maximum at one of those same ends.
            let ends = [low.x, high.x].map(|u| front + (back - front) * u);
            let mut axial = (f64::INFINITY, f64::NEG_INFINITY);
            let mut radius = 0.0f64;
            for end in ends {
                let offset = end - origin;
                if !offset.x.is_finite() || !offset.y.is_finite() || !offset.z.is_finite() {
                    return None;
                }
                let along = offset.dot(axis);
                axial = (axial.0.min(along), axial.1.max(along));
                radius = radius.max((offset - axis * along).magnitude());
            }
            Some(revolution_region_box(origin, axis, axial, radius))
        }
        // Sound, and simply blind to the trim.
        other => certified_surface_bounding_box(other, None),
    }
}

/// The overlap of two boxes, or [`None`] when they do not overlap.
///
/// An empty overlap means the two enclosures disagree, which is a contradiction
/// about geometry we are supposed to be certifying -- so it refuses rather than
/// pick a side.
fn intersect_boxes(
    first: BoundingBox<Point3>,
    second: BoundingBox<Point3>,
) -> Option<BoundingBox<Point3>> {
    let (first_low, first_high) = (first.min(), first.max());
    let (second_low, second_high) = (second.min(), second.max());
    let low = Point3::new(
        first_low.x.max(second_low.x),
        first_low.y.max(second_low.y),
        first_low.z.max(second_low.z),
    );
    let high = Point3::new(
        first_high.x.min(second_high.x),
        first_high.y.min(second_high.y),
        first_high.z.min(second_high.z),
    );
    (low.x <= high.x && low.y <= high.y && low.z <= high.z)
        .then(|| BoundingBox::from_iter([low, high]))
}

/// Pushes the eight corners of `bbox`; their convex hull is `bbox`.
fn push_box_corners(bbox: &BoundingBox<Point3>, points: &mut Vec<Point3>) {
    let (low, high) = (bbox.min(), bbox.max());
    for x in [low.x, high.x] {
        for y in [low.y, high.y] {
            for z in [low.z, high.z] {
                points.push(Point3::new(x, y, z));
            }
        }
    }
}

/// `(x/w, y/w, z/w)`, or [`None`] when the weight is not strictly positive --
/// the precondition of the rational convex-hull property.
fn projected_control_point(control: Vector4) -> Option<Point3> {
    let weight = control.w;
    (weight > 0.0 && weight.is_finite())
        .then(|| Point3::new(control.x / weight, control.y / weight, control.z / weight))
}

/// The box of a finite point set, or [`None`] when it is empty.
fn hull_box(points: &[Point3]) -> Option<BoundingBox<Point3>> {
    hull_box_iter(points.iter().copied())
}

/// The box of a finite point stream, or [`None`] when it is empty.
fn hull_box_iter(points: impl Iterator<Item = Point3>) -> Option<BoundingBox<Point3>> {
    let mut result = BoundingBox::<Point3>::new();
    let mut any = false;
    for point in points {
        if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() {
            return None;
        }
        result.push(point);
        any = true;
    }
    any.then_some(result)
}

/// The uniform scale factor of `matrix`, or [`None`] when it is not a
/// similarity (the only class of transform under which "revolution about an
/// axis" and "ball of radius r" survive as such).
fn similarity_scale(matrix: &Matrix4) -> Option<f64> {
    if matrix.x.w.abs() > SIMILARITY_TOL
        || matrix.y.w.abs() > SIMILARITY_TOL
        || matrix.z.w.abs() > SIMILARITY_TOL
        || (matrix.w.w - 1.0).abs() > SIMILARITY_TOL
    {
        return None;
    }
    let columns = [
        matrix.x.truncate(),
        matrix.y.truncate(),
        matrix.z.truncate(),
    ];
    let scale = columns[0].magnitude();
    if !scale.is_finite() || scale <= 0.0 {
        return None;
    }
    let length_tol = SIMILARITY_TOL * scale;
    let dot_tol = SIMILARITY_TOL * scale * scale;
    for column in &columns[1..] {
        if (column.magnitude() - scale).abs() > length_tol {
            return None;
        }
    }
    for (first, second) in [(0, 1), (0, 2), (1, 2)] {
        if columns[first].dot(columns[second]).abs() > dot_tol {
            return None;
        }
    }
    Some(scale)
}

/// `matrix` applied to a point, affine part included.
fn transform_point(matrix: &Matrix4, point: Point3) -> Point3 {
    let homogeneous = matrix * point.to_homogeneous();
    Point3::new(homogeneous.x, homogeneous.y, homogeneous.z)
}

/// `matrix` applied to a direction, renormalised, or [`None`] when it collapses.
fn transform_direction(matrix: &Matrix4, direction: Vector3) -> Option<Vector3> {
    let mapped = (matrix * direction.extend(0.0)).truncate();
    let length = mapped.magnitude();
    (length.is_finite() && length > 0.0).then(|| mapped / length)
}

/// The box of `{ origin + t * axis + w : t in axial, w perp axis, |w| <= radius }`.
///
/// Along a world axis `e` the extreme of `dot(w, e)` over that disc is
/// `radius * sqrt(1 - dot(axis, e)^2)`, which is what the `sqrt` below is.
fn revolution_region_box(
    origin: Point3,
    axis: Vector3,
    axial: (f64, f64),
    radius: f64,
) -> BoundingBox<Point3> {
    let (mut low, mut high) = ([0.0f64; 3], [0.0f64; 3]);
    for index in 0..3 {
        let along = axis[index];
        let (first, second) = (axial.0 * along, axial.1 * along);
        let perpendicular = radius * (1.0 - along * along).max(0.0).sqrt();
        low[index] = origin[index] + first.min(second) - perpendicular;
        high[index] = origin[index] + first.max(second) + perpendicular;
    }
    BoundingBox::from_iter([
        Point3::new(low[0], low[1], low[2]),
        Point3::new(high[0], high[1], high[2]),
    ])
}

/// The box of a cylinder/cone face, from points enclosing its boundary wires.
///
/// The boundary hull's axial span contains the face's, and its maximal radius
/// is at least the face's (the face's own extremes are attained on its
/// boundary, and the boundary lies in the hull) -- see the module note.
fn revolution_face_box(
    origin: Point3,
    axis: Vector3,
    boundary_hull: &[Point3],
) -> Option<BoundingBox<Point3>> {
    let (mut low, mut high, mut radius) = (f64::INFINITY, f64::NEG_INFINITY, 0.0f64);
    for point in boundary_hull {
        let offset = point - origin;
        if !offset.x.is_finite() || !offset.y.is_finite() || !offset.z.is_finite() {
            return None;
        }
        let along = offset.dot(axis);
        low = low.min(along);
        high = high.max(along);
        radius = radius.max((offset - axis * along).magnitude());
    }
    (low <= high && radius.is_finite())
        .then(|| revolution_region_box(origin, axis, (low, high), radius))
}

#[cfg(test)]
mod tests;
