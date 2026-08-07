use super::*;

mod accessors;
mod builders;
mod evaluation;
mod knots;
mod split;
mod traits;

impl<P: ControlPoint<f64>> BsplineSurface<P> {
    #[inline(always)]
    fn udelta_control_points(&self, i: usize, j: usize) -> P::Diff {
        if i == 0 {
            self.control_point(i, j).to_vec()
        } else if i == self.control_points.len() {
            self.control_point(i - 1, j).to_vec() * (-1.0)
        } else {
            *self.control_point(i, j) - *self.control_point(i - 1, j)
        }
    }

    #[inline(always)]
    fn vdelta_control_points(&self, i: usize, j: usize) -> P::Diff {
        if j == 0 {
            self.control_point(i, j).to_vec()
        } else if j == self.control_points[0].len() {
            self.control_point(i, j - 1).to_vec() * (-1.0)
        } else {
            *self.control_point(i, j) - *self.control_point(i, j - 1)
        }
    }

    pub(super) fn sub_near_as_surface<F: Fn(&P, &P) -> bool>(
        &self,
        other: &BsplineSurface<P>,
        div_coef: usize,
        ord: F,
    ) -> bool {
        if !self.knot_vecs.0.same_range(&other.knot_vecs.0) {
            return false;
        }
        if !self.knot_vecs.1.same_range(&other.knot_vecs.1) {
            return false;
        }

        let (self_degree0, self_degree1) = self.degrees();
        let (other_degree0, other_degree1) = other.degrees();
        let division0 = std::cmp::max(self_degree0, other_degree0) * div_coef;
        let division1 = std::cmp::max(self_degree1, other_degree1) * div_coef;

        for i0 in 1..self.knot_vecs.0.len() {
            let delta0 = self.knot_vecs.0[i0] - self.knot_vecs.0[i0 - 1];
            if delta0.so_small() {
                continue;
            }
            for j0 in 0..division0 {
                let u = self.knot_vecs.0[i0 - 1] + delta0 * (j0 as f64) / (division0 as f64);
                for i1 in 1..self.knot_vecs.1.len() {
                    let delta1 = self.knot_vecs.1[i1] - self.knot_vecs.1[i1 - 1];
                    if delta1.so_small() {
                        continue;
                    }
                    for j1 in 0..division1 {
                        let v =
                            self.knot_vecs.1[i1 - 1] + delta1 * (j1 as f64) / (division1 as f64);
                        if !ord(&self.subs(u, v), &other.subs(u, v)) {
                            return false;
                        }
                    }
                }
            }
        }
        true
    }
}

impl<P: ControlPoint<f64> + Tolerance> BsplineSurface<P> {
    /// Determines whether `self` and `other` is near as the B-spline surfaces or not.
    ///
    /// Divides each knot domain into the number of degree equal parts,
    /// and check `|self(u, v) - other(u, v)| < TOLERANCE` for each end points `(u, v)`.
    /// # Examples
    /// ```
    /// use monstertruck_geometry::prelude::*;
    /// let knot_vecs = (KnotVector::bezier_knot(3), KnotVector::bezier_knot(2));
    /// let control_points = vec![
    ///     vec![Vector2::new(0.0, 0.0), Vector2::new(0.5, -1.0), Vector2::new(1.0, 0.0)],
    ///     vec![Vector2::new(0.0, 1.0), Vector2::new(0.5, 1.0), Vector2::new(1.0, 1.0)],
    ///     vec![Vector2::new(0.0, 2.0), Vector2::new(0.5, 2.0), Vector2::new(1.0, 2.0)],
    ///     vec![Vector2::new(0.0, 3.0), Vector2::new(0.5, 3.5), Vector2::new(1.0, 3.0)],
    /// ];
    /// let bspsurface0 = BsplineSurface::new(knot_vecs, control_points);
    /// let mut bspsurface1 = bspsurface0.clone();
    /// assert!(bspsurface0.near_as_surface(&bspsurface1));
    ///
    /// *bspsurface1.control_point_mut(1, 1) = Vector2::new(0.4, 1.0);
    /// assert!(!bspsurface0.near_as_surface(&bspsurface1));
    /// ```
    #[inline(always)]
    pub fn near_as_surface(&self, other: &BsplineSurface<P>) -> bool {
        self.sub_near_as_surface(other, 1, |x, y| x.near(y))
    }
    /// Determines whether `self` and `other` is near in square order as the B-spline surfaces or not.
    ///
    /// Divides each knot domain into the number of degree equal parts,
    /// and check `|self(u, v) - other(u, v)| < TOLERANCE` for each end points `(u, v)`.
    /// # Examples
    /// ```
    /// use monstertruck_geometry::prelude::*;
    /// let eps = TOLERANCE;
    /// let knot_vecs = (KnotVector::bezier_knot(3), KnotVector::bezier_knot(2));
    /// let control_points = vec![
    ///     vec![Vector2::new(0.0, 0.0), Vector2::new(0.5, -1.0), Vector2::new(1.0, 0.0)],
    ///     vec![Vector2::new(0.0, 1.0), Vector2::new(0.5, 1.0), Vector2::new(1.0, 1.0)],
    ///     vec![Vector2::new(0.0, 2.0), Vector2::new(0.5, 2.0), Vector2::new(1.0, 2.0)],
    ///     vec![Vector2::new(0.0, 3.0), Vector2::new(0.5, 3.5), Vector2::new(1.0, 3.0)],
    /// ];
    /// let bspsurface0 = BsplineSurface::new(knot_vecs, control_points);
    /// let mut bspsurface1 = bspsurface0.clone();
    /// assert!(bspsurface0.near_as_surface(&bspsurface1));
    ///
    /// *bspsurface1.control_point_mut(1, 1) += Vector2::new(eps, eps / 2.0);
    /// assert!(bspsurface0.near_as_surface(&bspsurface1));
    /// assert!(!bspsurface0.near2_as_surface(&bspsurface1));
    /// ```
    #[inline(always)]
    pub fn near2_as_surface(&self, other: &BsplineSurface<P>) -> bool {
        self.sub_near_as_surface(other, 1, |x, y| x.near2(y))
    }
}

#[test]
fn test_include_bspcurve2() {
    let knot_vec = KnotVector::uniform_knot(2, 3);
    let control_points = vec![
        vec![
            Point2::new(0.0, 0.0),
            Point2::new(0.1, 0.0),
            Point2::new(0.5, 0.0),
            Point2::new(0.7, 0.0),
            Point2::new(1.0, 0.0),
        ],
        vec![
            Point2::new(0.0, 0.1),
            Point2::new(0.2, 0.2),
            Point2::new(0.4, 0.3),
            Point2::new(0.6, 0.2),
            Point2::new(1.0, 0.3),
        ],
        vec![
            Point2::new(0.0, 0.5),
            Point2::new(0.3, 0.6),
            Point2::new(0.6, 0.4),
            Point2::new(0.9, 0.6),
            Point2::new(1.0, 0.5),
        ],
        vec![
            Point2::new(0.0, 0.7),
            Point2::new(0.2, 0.8),
            Point2::new(0.3, 0.6),
            Point2::new(0.5, 0.9),
            Point2::new(1.0, 0.7),
        ],
        vec![
            Point2::new(0.0, 1.0),
            Point2::new(0.1, 1.0),
            Point2::new(0.5, 1.0),
            Point2::new(0.7, 1.0),
            Point2::new(1.0, 1.0),
        ],
    ];
    let surface = BsplineSurface::new((knot_vec.clone(), knot_vec), control_points);

    let knot_vec0 = KnotVector::bezier_knot(2);
    let control_points0 = vec![
        Point2::new(0.0, 0.0),
        Point2::new(1.0, 1.0),
        Point2::new(0.0, 1.0),
    ];
    let curve0 = BsplineCurve::new(knot_vec0, control_points0);
    assert!(surface.include(&curve0));

    let knot_vec1 = KnotVector::bezier_knot(2);
    let control_points1 = vec![
        Point2::new(0.0, 0.0),
        Point2::new(2.5, 1.0),
        Point2::new(0.0, 1.0),
    ];
    let curve1 = BsplineCurve::new(knot_vec1, control_points1);
    assert!(!surface.include(&curve1));
}

#[test]
fn test_include_bspcurve3() {
    let knot_vec = KnotVector::uniform_knot(2, 3);
    let control_points = vec![
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.1, 0.0, 0.5),
            Point3::new(0.5, 0.0, 0.3),
            Point3::new(1.0, 0.0, 1.0),
        ],
        vec![
            Point3::new(0.0, 0.1, 0.1),
            Point3::new(0.2, 0.2, 0.1),
            Point3::new(0.4, 0.3, 0.4),
            Point3::new(1.0, 0.3, 0.7),
        ],
        vec![
            Point3::new(0.0, 0.5, 0.4),
            Point3::new(0.3, 0.6, 0.5),
            Point3::new(0.6, 0.4, 1.0),
            Point3::new(1.0, 0.5, 0.0),
        ],
        vec![
            Point3::new(0.0, 1.0, 1.0),
            Point3::new(0.1, 1.0, 1.0),
            Point3::new(0.5, 1.0, 0.5),
            Point3::new(1.0, 1.0, 0.3),
        ],
    ];
    let surface = BsplineSurface::new((knot_vec.clone(), knot_vec), control_points);
    let bnd_box = BoundingBox::from_iter(&[Vector2::new(0.2, 0.3), Vector2::new(0.8, 0.6)]);
    let mut curve = surface.sectional_curve(bnd_box);
    assert!(surface.include(&curve));
    *curve.control_point_mut(2) += Vector3::new(0.0, 0.0, 0.001);
    assert!(!surface.include(&curve));
}

#[test]
fn cut_at_boundary_snapping_parameter_leaves_empty_net_but_does_not_panic() {
    // Cutting at a parameter that `near()`-snaps to a domain-start knot (but is
    // not exactly equal to it, so the fast path is skipped) makes `cut_u` slice
    // an empty `control_points[0..0]`. The subsequent `swap_axes` inside `cut_v`
    // then indexed `control_points[0]` on the empty net -- an out-of-bounds
    // panic. `swap_axes` must survive an empty control net gracefully.
    let mut surface = BsplineSurface::new(
        (KnotVector::bezier_knot(1), KnotVector::bezier_knot(1)),
        vec![
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
            vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
        ],
    );
    // 1.0e-6 is within TOLERANCE of the v-start knot (0.0); must not panic.
    let _ = surface.cut_v(1.0e-6);
}

#[test]
fn swap_axes_on_empty_control_net_swaps_knots_without_panicking() {
    // Defensive contract: `swap_axes` on a degenerate (zero-row) control net
    // swaps the knot vectors and leaves the net empty rather than indexing it.
    let mut surface: BsplineSurface<Point3> = BsplineSurface::new_unchecked(
        (
            KnotVector::from(vec![0.0, 0.0]),
            KnotVector::from(vec![0.0, 0.0, 1.0, 1.0]),
        ),
        Vec::new(),
    );
    surface.swap_axes();
    assert!(surface.control_points().is_empty());
    assert_eq!(surface.knot_vector_u().to_vec(), vec![0.0, 0.0, 1.0, 1.0]);
    assert_eq!(surface.knot_vector_v().to_vec(), vec![0.0, 0.0]);
}
