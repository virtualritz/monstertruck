//! The 3-D [`Surface`] enum and every impl that belongs to it.

use super::parameter_curves::same_surface;
use super::*;

/// 3-dimensional surfaces
#[derive(
    Clone,
    Debug,
    Serialize,
    Deserialize,
    From,
    ParametricSurface,
    ParameterDivision2D,
    Invertible,
    SearchParameterD2,
)]
#[allow(clippy::large_enum_variant)]
pub enum Surface {
    /// Plane
    Plane(Plane),
    /// 3-dimensional B-spline surface
    BsplineSurface(BsplineSurface<Point3>),
    /// 3-dimensional NURBS Surface
    NurbsSurface(NurbsSurface<Vector4>),
    /// revoluted curve
    #[serde(alias = "RevolutedCurve")]
    RevolutionSurface(Processor<RevolutionSurface<Curve>, Matrix4>),
    /// T-spline surface
    TsplineSurface(Tmesh<Point3>),
    /// Analytic sphere, posed by the processor's transform.
    ///
    /// Carried as the analytic type rather than its (exact) rational net so the
    /// CLOSED-FORM [`ParameterDivision2D`] survives the STEP conversion -- see
    /// the module note on `TryFrom<&Surface> for Surface` in
    /// `monstertruck-io/src/step/load/step_geometry/geom_impls/`. Spec 012 U1.2.
    SphericalSurface(Processor<Sphere, Matrix4>),
    /// Analytic torus, posed by the processor's transform. Same reason as
    /// [`Surface::SphericalSurface`]; spindle tori never reach this variant.
    ToroidalSurface(Processor<Torus, Matrix4>),
}

macro_rules! derive_surface_method {
    ($surface: expr, $method: expr, $($ver: ident),*) => {
        match $surface {
            Self::Plane(got) => $method(got, $($ver), *),
            Self::BsplineSurface(got) => $method(got, $($ver), *),
            Self::NurbsSurface(got) => $method(got, $($ver), *),
            Self::RevolutionSurface(got) => $method(got, $($ver), *),
            Self::TsplineSurface(got) => $method(got, $($ver), *),
            Self::SphericalSurface(got) => $method(got, $($ver), *),
            Self::ToroidalSurface(got) => $method(got, $($ver), *),
        }
    };
}

macro_rules! derive_surface_self_method {
    ($surface: expr, $method: expr, $($ver: ident),*) => {
        match $surface {
            Self::Plane(got) => Self::Plane($method(got, $($ver), *)),
            Self::BsplineSurface(got) => Self::BsplineSurface($method(got, $($ver), *)),
            Self::NurbsSurface(got) => Self::NurbsSurface($method(got, $($ver), *)),
            Self::RevolutionSurface(got) => Self::RevolutionSurface($method(got, $($ver), *)),
            Self::TsplineSurface(got) => Self::TsplineSurface($method(got, $($ver), *)),
            Self::SphericalSurface(got) => Self::SphericalSurface($method(got, $($ver), *)),
            Self::ToroidalSurface(got) => Self::ToroidalSurface($method(got, $($ver), *)),
        }
    };
}

impl ParametricSurface3D for Surface {
    #[inline(always)]
    fn normal(&self, u: f64, v: f64) -> Vector3 {
        derive_surface_method!(self, ParametricSurface3D::normal, u, v)
    }
}

/// Trials the analytic containment test allows its inverse map, matching
/// `INCLUDE_CURVE_TRIALS` in `monstertruck-geometry`'s `NurbsSurface` impl --
/// the one [`Surface::SphericalSurface`] and [`Surface::ToroidalSurface`]
/// faces reached before spec 012 U1.2 gave them analytic variants.
const ANALYTIC_INCLUDE_CURVE_TRIALS: usize = 100;

/// Whether every sample of `curve` recovers a parameter on `surface`.
///
/// Samples the CURVE at the same density the `NurbsSurface` impl this replaces
/// used -- interior points of each knot span, `2 * degree` of them -- rather
/// than the control polygon, whose points are not on a rational curve at all.
fn analytic_surface_includes_curve<S>(surface: &S, curve: &Curve) -> bool
where S: SearchParameter<SurfaceParameter, Point = Point3> {
    let lifted = NurbsCurve::new(curve.lift_up());
    let (knots, _) = lifted.knot_vector().to_single_multi();
    let degree = usize::max(lifted.degree() * 2, 2);
    knots.windows(2).all(|window| {
        (0..=degree).all(|index| {
            let ratio = index as f64 / degree as f64;
            let parameter = window[0] * (1.0 - ratio) + window[1] * ratio;
            surface
                .search_parameter(lifted.subs(parameter), None, ANALYTIC_INCLUDE_CURVE_TRIALS)
                .is_some()
        })
    })
}

impl Transformed<Matrix4> for Surface {
    fn transform_by(&mut self, trans: Matrix4) {
        derive_surface_method!(self, Transformed::transform_by, trans);
    }
    fn transformed(&self, trans: Matrix4) -> Self {
        derive_surface_self_method!(self, Transformed::transformed, trans)
    }
}

impl IncludeCurve<Curve> for Surface {
    #[inline(always)]
    fn include(&self, curve: &Curve) -> bool {
        if let Curve::ParameterCurve(curve) = curve {
            same_surface(curve.surface().as_ref(), self)
        } else {
            match self {
                Surface::BsplineSurface(surface) => match curve {
                    &Curve::Line(curve) => surface.include(&BsplineCurve::from(curve)),
                    Curve::BsplineCurve(curve) => surface.include(curve),
                    Curve::NurbsCurve(curve) => surface.include(curve),
                    Curve::IntersectionCurve(curve) => match curve.leader().as_ref() {
                        Curve::Line(curve) => surface.include(&BsplineCurve::from(*curve)),
                        Curve::BsplineCurve(curve) => surface.include(curve),
                        Curve::NurbsCurve(curve) => surface.include(curve),
                        Curve::ParameterCurve(_) => false,
                        Curve::IntersectionCurve(_) => false,
                    },
                    Curve::ParameterCurve(_) => unreachable!(),
                },
                Surface::NurbsSurface(surface) => match curve {
                    &Curve::Line(curve) => surface.include(&BsplineCurve::from(curve)),
                    Curve::BsplineCurve(curve) => surface.include(curve),
                    Curve::NurbsCurve(curve) => surface.include(curve),
                    Curve::IntersectionCurve(curve) => match curve.leader().as_ref() {
                        Curve::Line(curve) => surface.include(&BsplineCurve::from(*curve)),
                        Curve::BsplineCurve(curve) => surface.include(curve),
                        Curve::NurbsCurve(curve) => surface.include(curve),
                        Curve::ParameterCurve(_) => false,
                        Curve::IntersectionCurve(_) => false,
                    },
                    Curve::ParameterCurve(_) => unreachable!(),
                },
                Surface::Plane(surface) => match curve {
                    &Curve::Line(curve) => surface.include(&BsplineCurve::from(curve)),
                    Curve::BsplineCurve(curve) => surface.include(curve),
                    Curve::NurbsCurve(curve) => surface.include(curve),
                    Curve::IntersectionCurve(curve) => match curve.leader().as_ref() {
                        Curve::Line(curve) => surface.include(&BsplineCurve::from(*curve)),
                        Curve::BsplineCurve(curve) => surface.include(curve),
                        Curve::NurbsCurve(curve) => surface.include(curve),
                        Curve::ParameterCurve(_) => false,
                        Curve::IntersectionCurve(_) => false,
                    },
                    Curve::ParameterCurve(_) => unreachable!(),
                },
                Surface::TsplineSurface(surface) => {
                    curve.lift_up().control_points().iter().all(|v| {
                        let p = v.to_point();
                        surface.search_parameter(p, None, 1).is_some()
                    })
                }
                // The analytic variants answer containment through their own
                // exact inverse map. `Torus` carries no `IncludeCurve` impl at
                // all, so the pair shares one sampled test -- and it samples
                // the CURVE, like the `NurbsSurface` impl these faces used to
                // reach, not the control polygon like the T-mesh arm above. A
                // rational curve's control points are not on the curve, so
                // testing them would have been strictly more conservative than
                // what this replaces.
                Surface::SphericalSurface(surface) => {
                    analytic_surface_includes_curve(surface, curve)
                }
                Surface::ToroidalSurface(surface) => {
                    analytic_surface_includes_curve(surface, curve)
                }
                Surface::RevolutionSurface(surface) => match surface.entity_curve() {
                    &Curve::Line(entity_line) => {
                        let entity_bsp = BsplineCurve::from(entity_line);
                        let surface = RevolutionSurface::by_revolution(
                            &entity_bsp,
                            surface.origin(),
                            surface.axis(),
                        );
                        match curve {
                            &Curve::Line(curve) => surface.include(&BsplineCurve::from(curve)),
                            Curve::BsplineCurve(curve) => surface.include(curve),
                            Curve::NurbsCurve(curve) => surface.include(curve),
                            Curve::IntersectionCurve(curve) => match curve.leader().as_ref() {
                                Curve::Line(curve) => surface.include(&BsplineCurve::from(*curve)),
                                Curve::BsplineCurve(curve) => surface.include(curve),
                                Curve::NurbsCurve(curve) => surface.include(curve),
                                Curve::ParameterCurve(_) => false,
                                Curve::IntersectionCurve(_) => false,
                            },
                            Curve::ParameterCurve(_) => unreachable!(),
                        }
                    }
                    Curve::BsplineCurve(entity_curve) => {
                        let surface = RevolutionSurface::by_revolution(
                            entity_curve,
                            surface.origin(),
                            surface.axis(),
                        );
                        match curve {
                            &Curve::Line(curve) => surface.include(&BsplineCurve::from(curve)),
                            Curve::BsplineCurve(curve) => surface.include(curve),
                            Curve::NurbsCurve(curve) => surface.include(curve),
                            Curve::IntersectionCurve(curve) => match curve.leader().as_ref() {
                                Curve::Line(curve) => surface.include(&BsplineCurve::from(*curve)),
                                Curve::BsplineCurve(curve) => surface.include(curve),
                                Curve::NurbsCurve(curve) => surface.include(curve),
                                Curve::ParameterCurve(_) => false,
                                Curve::IntersectionCurve(_) => false,
                            },
                            Curve::ParameterCurve(_) => unreachable!(),
                        }
                    }
                    Curve::NurbsCurve(entity_curve) => {
                        let surface = RevolutionSurface::by_revolution(
                            entity_curve,
                            surface.origin(),
                            surface.axis(),
                        );
                        match curve {
                            &Curve::Line(curve) => surface.include(&BsplineCurve::from(curve)),
                            Curve::BsplineCurve(curve) => surface.include(curve),
                            Curve::NurbsCurve(curve) => surface.include(curve),
                            Curve::IntersectionCurve(curve) => match curve.leader().as_ref() {
                                Curve::Line(curve) => surface.include(&BsplineCurve::from(*curve)),
                                Curve::BsplineCurve(curve) => surface.include(curve),
                                Curve::NurbsCurve(curve) => surface.include(curve),
                                Curve::ParameterCurve(_) => false,
                                Curve::IntersectionCurve(_) => false,
                            },
                            Curve::ParameterCurve(_) => unreachable!(),
                        }
                    }
                    Curve::IntersectionCurve(entity_curve) => {
                        let leader = entity_curve.leader().as_ref();
                        match leader {
                            Curve::Line(entity_line) => {
                                let entity_bsp = BsplineCurve::from(*entity_line);
                                let surface = RevolutionSurface::by_revolution(
                                    &entity_bsp,
                                    surface.origin(),
                                    surface.axis(),
                                );
                                match curve {
                                    &Curve::Line(curve) => {
                                        surface.include(&BsplineCurve::from(curve))
                                    }
                                    Curve::BsplineCurve(curve) => surface.include(curve),
                                    Curve::NurbsCurve(curve) => surface.include(curve),
                                    Curve::IntersectionCurve(curve) => {
                                        match curve.leader().as_ref() {
                                            Curve::Line(curve) => {
                                                surface.include(&BsplineCurve::from(*curve))
                                            }
                                            Curve::BsplineCurve(curve) => surface.include(curve),
                                            Curve::NurbsCurve(curve) => surface.include(curve),
                                            Curve::ParameterCurve(_) => false,
                                            Curve::IntersectionCurve(_) => false,
                                        }
                                    }
                                    Curve::ParameterCurve(_) => unreachable!(),
                                }
                            }
                            Curve::BsplineCurve(entity_curve) => {
                                let surface = RevolutionSurface::by_revolution(
                                    entity_curve,
                                    surface.origin(),
                                    surface.axis(),
                                );
                                match curve {
                                    &Curve::Line(curve) => {
                                        surface.include(&BsplineCurve::from(curve))
                                    }
                                    Curve::BsplineCurve(curve) => surface.include(curve),
                                    Curve::NurbsCurve(curve) => surface.include(curve),
                                    Curve::IntersectionCurve(curve) => {
                                        match curve.leader().as_ref() {
                                            Curve::Line(curve) => {
                                                surface.include(&BsplineCurve::from(*curve))
                                            }
                                            Curve::BsplineCurve(curve) => surface.include(curve),
                                            Curve::NurbsCurve(curve) => surface.include(curve),
                                            Curve::ParameterCurve(_) => false,
                                            Curve::IntersectionCurve(_) => false,
                                        }
                                    }
                                    Curve::ParameterCurve(_) => unreachable!(),
                                }
                            }
                            Curve::NurbsCurve(entity_curve) => {
                                let surface = RevolutionSurface::by_revolution(
                                    entity_curve,
                                    surface.origin(),
                                    surface.axis(),
                                );
                                match curve {
                                    &Curve::Line(curve) => {
                                        surface.include(&BsplineCurve::from(curve))
                                    }
                                    Curve::BsplineCurve(curve) => surface.include(curve),
                                    Curve::NurbsCurve(curve) => surface.include(curve),
                                    Curve::IntersectionCurve(curve) => {
                                        match curve.leader().as_ref() {
                                            Curve::Line(curve) => {
                                                surface.include(&BsplineCurve::from(*curve))
                                            }
                                            Curve::BsplineCurve(curve) => surface.include(curve),
                                            Curve::NurbsCurve(curve) => surface.include(curve),
                                            Curve::ParameterCurve(_) => false,
                                            Curve::IntersectionCurve(_) => false,
                                        }
                                    }
                                    Curve::ParameterCurve(_) => unreachable!(),
                                }
                            }
                            Curve::ParameterCurve(_) => false,
                            Curve::IntersectionCurve(_) => false,
                        }
                    }
                    Curve::ParameterCurve(_) => false,
                },
            }
        }
    }
}

impl IncludeCurve<Curve> for Plane {
    fn include(&self, curve: &Curve) -> bool {
        curve.lift_up().control_points().iter().all(|v| {
            let p = v.to_point();
            self.search_parameter(p, None, 1).is_some()
        })
    }
}

impl ToSameGeometry<Surface> for Plane {
    fn to_same_geometry(&self) -> Surface { (*self).into() }
}

impl ToSameGeometry<Surface> for RevolutionSurface<Curve> {
    fn to_same_geometry(&self) -> Surface {
        Surface::RevolutionSurface(Processor::new(self.clone()))
    }
}

fn transform_preserves_nearest_parameter(transform: &Matrix4) -> bool {
    let tol = 1.0e-10;
    let x = Vector3::new(transform.x.x, transform.x.y, transform.x.z);
    let y = Vector3::new(transform.y.x, transform.y.y, transform.y.z);
    let z = Vector3::new(transform.z.x, transform.z.y, transform.z.z);
    let scale2 = x.magnitude2();
    scale2 > tol
        && f64::abs(y.magnitude2() - scale2) <= tol
        && f64::abs(z.magnitude2() - scale2) <= tol
        && f64::abs(x.dot(y)) <= tol
        && f64::abs(x.dot(z)) <= tol
        && f64::abs(y.dot(z)) <= tol
        && f64::abs(transform.x.w) <= tol
        && f64::abs(transform.y.w) <= tol
        && f64::abs(transform.z.w) <= tol
        && f64::abs(transform.w.w - 1.0) <= tol
}

fn revolution_surface_nearest_parameter(
    surface: &Processor<RevolutionSurface<Curve>, Matrix4>,
    point: Point3,
    hint: SearchParameterHint2D,
    trials: usize,
) -> Option<(f64, f64)> {
    if env::var("MT_BOOL_DISABLE_REVOLUTION_NEAREST_FAST_PATH").is_err()
        && transform_preserves_nearest_parameter(surface.transform())
    {
        let inv = surface.transform().inverse_transform()?;
        let point = inv.transform_point(point);
        let uv = surface
            .entity()
            .search_nearest_parameter(point, hint, trials)?;
        Some(if surface.orientation() {
            uv
        } else {
            (uv.1, uv.0)
        })
    } else {
        surface.search_nearest_parameter(point, hint, trials)
    }
}

/// Separable presearch for a revolved-curve surface, bit-identical to the generic
/// [`algo::surface::presearch`] over the same `Processor<RevolutionSurface<..>, Matrix4>`.
///
/// `Processor::evaluate(u, v)` expands (per `orientation`) to
/// `transform(origin + rotation(angle) * (curve(param) - origin))`, where the
/// NURBS/basis `curve(param)` evaluation depends only on one grid axis and the
/// sincos-built `rotation(angle)` matrix only on the other. The generic grid scan
/// re-derives both `division + 1` times per grid line; this hoists each out of the
/// `O(division^2)` inner loop, computing every distinct `curve(param)` and
/// `rotation(angle)` exactly once and reusing it across the crossing line -- the
/// same waste `NurbsSurface::presearch_separable` removes for tensor-product NURBS,
/// which never covered the revolved-curve surface path.
///
/// Everything that decides the result is unchanged: the same grid nodes (`u`, `v`
/// from the identical expressions), the identical evaluation arithmetic
/// (`transform.transform_point(origin + rotation * (curve - origin))`), the same
/// `distance2` metric, and the same strict-`<` first-minimum tie-break. The
/// returned `(u, v)` -- and therefore every downstream Newton seed -- is
/// byte-for-byte identical to the generic presearch.
fn revolution_processor_presearch_separable(
    rotted: &Processor<RevolutionSurface<Curve>, Matrix4>,
    point: Point3,
    (urange, vrange): ((f64, f64), (f64, f64)),
    division: usize,
) -> (f64, f64) {
    let revolution = rotted.entity();
    let transform = rotted.transform();
    let origin = revolution.origin();
    let axis = revolution.axis();
    let curve = revolution.entity_curve();
    // Spec 014 W3: the same grid as `algo::surface::presearch`, charged in the
    // same unit. This path presearches at division 100 where the tensor-product
    // paths use 50, so nodes -- unlike a count of CALLS -- can tell a revolution
    // search apart from a NURBS one (10,201 vs 2,601).
    algo::surface::charge_presearch_nodes(algo::surface::presearch_nodes(division));
    let ((u0, u1), (v0, v1)) = (urange, vrange);
    // Identical to the generic presearch's grid-node expression.
    let node = |bound0: f64, bound1: f64, index: usize| {
        let t = index as f64 / division as f64;
        bound0 * (1.0 - t) + bound1 * t
    };
    // Each expensive separable factor is derived once per grid line and reused
    // across the crossing line (the hoist), then paired with its node value so the
    // loops iterate by value -- no range-indexing. `Processor::evaluate(u, v)` feeds
    // (curve arg, rotation arg) = (u, v) when `orientation`, else (v, u), so the two
    // factors bind to opposite axes per orientation; the returned node and the
    // outer=`u`/inner=`v` iteration order stay fixed to reproduce the generic
    // presearch's argmin and strict-`<` first-minimum tie-break exactly.
    let mut res = (0.0, 0.0);
    let mut min = f64::INFINITY;
    if rotted.orientation() {
        // evaluate(u, v) = transform(origin + rotation(v) * (curve(u) - origin)).
        let curve_by_u: Vec<(f64, Point3)> = (0..=division)
            .map(|i| {
                let u = node(u0, u1, i);
                (u, curve.evaluate(u))
            })
            .collect();
        let rotation_by_v: Vec<(f64, Matrix3)> = (0..=division)
            .map(|j| {
                let v = node(v0, v1, j);
                (v, Matrix3::from_axis_angle(axis, Rad(v)))
            })
            .collect();
        for &(u, curve_point) in &curve_by_u {
            for &(v, rotation) in &rotation_by_v {
                let dist = transform
                    .transform_point(origin + rotation * (curve_point - origin))
                    .distance2(point);
                if dist < min {
                    min = dist;
                    res = (u, v);
                }
            }
        }
    } else {
        // evaluate(u, v) = transform(origin + rotation(u) * (curve(v) - origin)).
        let rotation_by_u: Vec<(f64, Matrix3)> = (0..=division)
            .map(|i| {
                let u = node(u0, u1, i);
                (u, Matrix3::from_axis_angle(axis, Rad(u)))
            })
            .collect();
        let curve_by_v: Vec<(f64, Point3)> = (0..=division)
            .map(|j| {
                let v = node(v0, v1, j);
                (v, curve.evaluate(v))
            })
            .collect();
        for &(u, rotation) in &rotation_by_u {
            for &(v, curve_point) in &curve_by_v {
                let dist = transform
                    .transform_point(origin + rotation * (curve_point - origin))
                    .distance2(point);
                if dist < min {
                    min = dist;
                    res = (u, v);
                }
            }
        }
    }
    res
}

impl SearchNearestParameter<SurfaceParameter> for Surface {
    type Point = Point3;
    fn search_nearest_parameter<H: Into<SearchParameterHint2D>>(
        &self,
        point: Point3,
        hint: H,
        trials: usize,
    ) -> Option<(f64, f64)> {
        match self {
            Surface::Plane(plane) => plane.search_nearest_parameter(point, hint, trials),
            Surface::BsplineSurface(bspsurface) => {
                bspsurface.search_nearest_parameter(point, hint, trials)
            }
            Surface::NurbsSurface(surface) => surface.search_nearest_parameter(point, hint, trials),
            Surface::TsplineSurface(surface) => {
                surface.search_nearest_parameter(point, hint, trials)
            }
            // The analytic inverse map, not a Newton descent. This is the half
            // of the sphere/torus conversion that the rational net never
            // carried, and (011-T1) the half that decides where trims land.
            Surface::SphericalSurface(surface) => {
                surface.search_nearest_parameter(point, hint, trials)
            }
            Surface::ToroidalSurface(surface) => {
                surface.search_nearest_parameter(point, hint, trials)
            }
            Surface::RevolutionSurface(rotted) => {
                let hint = hint.into();
                revolution_surface_nearest_parameter(rotted, point, hint, trials).or_else(|| {
                    let hint = match hint {
                        SearchParameterHint2D::Parameter(hint0, hint1) => (hint0, hint1),
                        SearchParameterHint2D::Range(x, y) => {
                            revolution_processor_presearch_separable(rotted, point, (x, y), 100)
                        }
                        SearchParameterHint2D::None => revolution_processor_presearch_separable(
                            rotted,
                            point,
                            rotted.range_tuple(),
                            100,
                        ),
                    };
                    algo::surface::search_nearest_parameter(rotted, point, hint, trials).or_else(
                        || {
                            let candidate = rotted.evaluate(hint.0, hint.1);
                            candidate.near(&point).then_some(hint)
                        },
                    )
                })
            }
        }
    }
}

impl TryIntoBsplineSurface for Surface {
    fn try_into_bspline_surface(&self) -> Option<BsplineSurface<Point3>> {
        match self {
            Surface::Plane(p) => p.try_into_bspline_surface(),
            Surface::BsplineSurface(b) => b.try_into_bspline_surface(),
            Surface::NurbsSurface(n) => n.try_into_bspline_surface(),
            Surface::RevolutionSurface(_) => None,
            Surface::TsplineSurface(_) => None,
            // No exact NON-rational net exists for either; the exact form is
            // the homogeneous one below.
            Surface::SphericalSurface(_) | Surface::ToroidalSurface(_) => None,
        }
    }
}

impl TryIntoHomogeneousBsplineSurface for Surface {
    fn try_into_homogeneous_bspline_surface(&self) -> Option<BsplineSurface<Vector4>> {
        match self {
            Surface::Plane(p) => p.try_into_homogeneous_bspline_surface(),
            Surface::BsplineSurface(b) => b.try_into_homogeneous_bspline_surface(),
            Surface::NurbsSurface(n) => n.try_into_homogeneous_bspline_surface(),
            Surface::RevolutionSurface(r) => r.try_into_homogeneous_bspline_surface(),
            Surface::TsplineSurface(_) => None,
            // The SAME call the STEP loader used to make eagerly, on the same
            // `Processor<_, Matrix4>`: the net the boolean's homogeneous path
            // prepares is byte-identical to the one it saw before this
            // variant existed. Only WHEN it is built has moved.
            Surface::SphericalSurface(s) => s.try_into_homogeneous_bspline_surface(),
            Surface::ToroidalSurface(t) => t.try_into_homogeneous_bspline_surface(),
        }
    }

    fn try_into_homogeneous_bspline_surface_over(
        &self,
        parameter_range: Option<SurfaceParameterRectangle>,
    ) -> Option<HomogeneousSurfaceConversion> {
        match self {
            Surface::Plane(p) => p.try_into_homogeneous_bspline_surface_over(parameter_range),
            Surface::BsplineSurface(b) => {
                b.try_into_homogeneous_bspline_surface_over(parameter_range)
            }
            Surface::NurbsSurface(n) => {
                n.try_into_homogeneous_bspline_surface_over(parameter_range)
            }
            Surface::RevolutionSurface(r) => {
                r.try_into_homogeneous_bspline_surface_over(parameter_range)
            }
            Surface::TsplineSurface(_) => None,
            Surface::SphericalSurface(s) => {
                s.try_into_homogeneous_bspline_surface_over(parameter_range)
            }
            Surface::ToroidalSurface(t) => {
                t.try_into_homogeneous_bspline_surface_over(parameter_range)
            }
        }
    }
}

impl SupportsExactPatchDomains for Surface {
    fn supports_exact_patch_domains(&self) -> bool {
        match self {
            Surface::Plane(p) => p.supports_exact_patch_domains(),
            Surface::BsplineSurface(b) => b.supports_exact_patch_domains(),
            Surface::NurbsSurface(n) => n.supports_exact_patch_domains(),
            Surface::RevolutionSurface(r) => r.supports_exact_patch_domains(),
            Surface::TsplineSurface(t) => t.supports_exact_patch_domains(),
            // `false`, where the rational net answered `true`. This is the same
            // flip T22 made when cylinders and cones moved onto the analytic
            // revolution variant, and for the same reason: the patch domain of
            // an analytic surface is not a knot span.
            Surface::SphericalSurface(s) => s.supports_exact_patch_domains(),
            Surface::ToroidalSurface(t) => t.supports_exact_patch_domains(),
        }
    }
}

impl TryIntoAnalyticSurfaceKind for Surface {
    fn try_into_analytic_surface_kind(&self) -> Option<AnalyticSurfaceKind> {
        match self {
            Surface::Plane(p) => p.try_into_analytic_surface_kind(),
            Surface::BsplineSurface(b) => b.try_into_analytic_surface_kind(),
            Surface::NurbsSurface(n) => n.try_into_analytic_surface_kind(),
            Surface::RevolutionSurface(r) => r.try_into_analytic_surface_kind(),
            Surface::TsplineSurface(t) => t.try_into_analytic_surface_kind(),
            // `None`, which is exactly what these faces answered as rational
            // nets: `NurbsSurface::try_into_analytic_surface_kind` only ever
            // recognises a plane or a homogeneous extrusion, and a sphere net
            // is neither. Teaching it `SphericalRevolution` would be a
            // BOOLEAN change, and this track is the tessellation one --
            // recorded as follow-on work instead of smuggled in here.
            Surface::SphericalSurface(s) => s.try_into_analytic_surface_kind(),
            Surface::ToroidalSurface(t) => t.try_into_analytic_surface_kind(),
        }
    }
}

impl ToSameGeometry<Surface> for HomotopySurface<Curve, Curve> {
    fn to_same_geometry(&self) -> Surface {
        let curve0 = self.first_curve().clone().lift_up();
        let curve1 = self.second_curve().clone().lift_up();
        NurbsSurface::new(BsplineSurface::homotopy(curve0, curve1)).into()
    }
}

impl ToSameGeometry<Surface> for ExtrusionSurface<Curve, Vector3> {
    fn to_same_geometry(&self) -> Surface {
        let (curve0, vector) = (self.entity_curve(), self.extruding_vector());
        let trsl = Matrix4::from_translation(vector);
        let curve1 = self.entity_curve().transformed(trsl);
        match (curve0, curve1) {
            (Curve::Line(line), Curve::Line(_)) => {
                Plane::new(line.0, line.1, line.0 + vector).into()
            }
            (Curve::BsplineCurve(curve0), Curve::BsplineCurve(curve1)) => {
                BsplineSurface::homotopy(curve0.clone(), curve1.clone()).into()
            }
            (Curve::NurbsCurve(curve0), Curve::NurbsCurve(curve1)) => {
                NurbsSurface::new(BsplineSurface::homotopy(
                    curve0.non_rationalized().clone(),
                    curve1.non_rationalized().clone(),
                ))
                .into()
            }
            (Curve::IntersectionCurve(_), Curve::IntersectionCurve(_)) => unimplemented!(),
            _ => unreachable!(),
        }
    }
}
