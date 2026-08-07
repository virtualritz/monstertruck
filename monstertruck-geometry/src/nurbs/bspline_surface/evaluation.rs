use super::*;

impl<P: ControlPoint<f64>> BsplineSurface<P> {
    /// Returns the closure of substitution.
    #[inline(always)]
    pub fn closure(&self) -> impl Fn(f64, f64) -> P + '_ { move |u, v| self.subs(u, v) }

    /// Calculate derived B-spline surface by the first parameter `u`.
    /// # Examples
    /// ```
    /// use monstertruck_geometry::prelude::*;
    /// let knot_vecs = (KnotVector::bezier_knot(1), KnotVector::bezier_knot(2));
    /// let control_points = vec![
    ///     vec![Vector2::new(0.0, 0.0), Vector2::new(0.5, -1.0), Vector2::new(1.0, 0.0)],
    ///     vec![Vector2::new(0.0, 1.0), Vector2::new(0.5, 2.0), Vector2::new(1.0, 1.0)],
    /// ];
    /// let bspsurface = BsplineSurface::new(knot_vecs, control_points);
    /// let uderivation = bspsurface.uderivation();
    ///
    /// // bspsurface: (v, 2v(1 - v)(2u - 1) + u), uderivation: (0.0, 4v(1 - v) + 1)
    /// const N: usize = 100; // sample size
    /// for i in 1..N {
    ///     let u = (i as f64) / (N as f64);
    ///     for j in 0..=N {
    ///         let v = (j as f64) / (N as f64);
    ///         assert_near2!(
    ///             uderivation.subs(u, v),
    ///             Vector2::new(0.0, 4.0 * v * (1.0 - v) + 1.0),
    ///         );
    ///     }
    /// }
    /// ```
    pub fn uderivation(&self) -> BsplineSurface<P::Diff> {
        let n0 = self.control_points.len();
        let n1 = self.control_points[0].len();
        let (k, _) = self.degrees();
        let (knot_vector_u, knot_vector_v) = self.knot_vecs.clone();

        let new_points = if k > 0 {
            (0..=n0)
                .map(|i| {
                    let delta = knot_vector_u[i + k] - knot_vector_u[i];
                    let coef = (k as f64) * inv_or_zero(delta);
                    (0..n1)
                        .map(|j| self.udelta_control_points(i, j) * coef)
                        .collect()
                })
                .collect()
        } else {
            vec![vec![P::Diff::zero(); n1]; n0]
        };

        BsplineSurface::new_unchecked((knot_vector_u, knot_vector_v), new_points)
    }

    /// Calculate derived B-spline surface by the second parameter `v`.
    /// # Examples
    /// ```
    /// use monstertruck_geometry::prelude::*;
    /// let knot_vecs = (KnotVector::bezier_knot(1), KnotVector::bezier_knot(2));
    /// let control_points = vec![
    ///     vec![Vector2::new(0.0, 0.0), Vector2::new(0.5, -1.0), Vector2::new(1.0, 0.0)],
    ///     vec![Vector2::new(0.0, 1.0), Vector2::new(0.5, 2.0), Vector2::new(1.0, 1.0)],
    /// ];
    /// let bspsurface = BsplineSurface::new(knot_vecs, control_points);
    /// let vderivation = bspsurface.vderivation();
    ///
    /// // bspsurface: (v, 2v(1 - v)(2u - 1) + u), vderivation: (1, -2(2u - 1)(2v - 1))
    /// const N: usize = 100; // sample size
    /// for i in 0..=N {
    ///     let u = (i as f64) / (N as f64);
    ///     for j in 0..=N {
    ///         let v = (j as f64) / (N as f64);
    ///         assert_near2!(
    ///             vderivation.subs(u, v),
    ///             Vector2::new(1.0, -2.0 * (2.0 * u - 1.0) * (2.0 * v - 1.0)),
    ///         );
    ///     }
    /// }
    /// ```
    pub fn vderivation(&self) -> BsplineSurface<P::Diff> {
        let n0 = self.control_points.len();
        let n1 = self.control_points[0].len();
        let (_, k) = self.degrees();

        let (knot_vector_u, knot_vector_v) = self.knot_vecs.clone();

        let new_points = if k > 0 {
            let mut new_points = vec![Vec::with_capacity(n1 + 1); n0];
            for j in 0..=n1 {
                let delta = knot_vector_v[j + k] - knot_vector_v[j];
                let coef = (k as f64) * inv_or_zero(delta);
                for (i, vec) in new_points.iter_mut().enumerate() {
                    vec.push(self.vdelta_control_points(i, j) * coef)
                }
            }
            new_points
        } else {
            vec![vec![P::Diff::zero(); n1]; n0]
        };

        BsplineSurface::new_unchecked((knot_vector_u, knot_vector_v), new_points)
    }
}

impl<P: ControlPoint<f64>> BsplineSurface<P> {
    /// Contracts the (already computed) `u`- and `v`-direction B-spline basis
    /// windows against the control net -- the bilinear de Boor sum that is the
    /// inner kernel of the non-degenerate branch of
    /// [`derivative_mn`](ParametricSurface::derivative_mn).
    ///
    /// Factored out so a separable grid evaluation can compute each row/column
    /// basis window once and reuse it across a whole grid line, instead of
    /// recomputing the same window per grid point. The summation is byte-for-byte
    /// the same as the inline `derivative_mn` fold, so results are bit-identical.
    #[inline]
    pub(crate) fn combine_basis(&self, basis0: &BasisWindow, basis1: &BasisWindow) -> P::Diff {
        let v_start = basis1.start_index();
        self.control_points[basis0.start_index()..]
            .iter()
            .zip(basis0.values())
            .fold(P::Diff::zero(), |sum, (row, &b0)| {
                let local = row[v_start..]
                    .iter()
                    .zip(basis1.values())
                    .fold(P::Diff::zero(), |sum, (&point, &b1)| {
                        sum + point.to_vec() * (b0 * b1)
                    });
                sum + local
            })
    }
}

impl<P: ControlPoint<f64>> ParametricSurface for BsplineSurface<P> {
    type Point = P;
    type Vector = P::Diff;
    fn derivative_mn(&self, m: usize, n: usize, u: f64, v: f64) -> Self::Vector {
        let (degree0, degree1) = self.degrees();
        let BsplineSurface {
            knot_vecs: (knot_vector_u, knot_vector_v),
            control_points,
        } = self;
        let u_range_is_zero = knot_vector_u[0] == knot_vector_u[knot_vector_u.len() - 1];
        let v_range_is_zero = knot_vector_v[0] == knot_vector_v[knot_vector_v.len() - 1];

        if u_range_is_zero && v_range_is_zero {
            if m == 0 && n == 0 {
                control_points[0][0].to_vec()
            } else {
                P::Diff::zero()
            }
        } else if u_range_is_zero {
            if m == 0 {
                let basis1 = knot_vector_v.bspline_basis_functions(degree1, n, v);
                control_points[0][basis1.start_index()..]
                    .iter()
                    .zip(basis1.values())
                    .fold(P::Diff::zero(), |sum, (&point, &b)| {
                        sum + point.to_vec() * b
                    })
            } else {
                P::Diff::zero()
            }
        } else if v_range_is_zero {
            if n == 0 {
                let basis0 = knot_vector_u.bspline_basis_functions(degree0, m, u);
                control_points[basis0.start_index()..]
                    .iter()
                    .zip(basis0.values())
                    .fold(P::Diff::zero(), |sum, (row, &b)| sum + row[0].to_vec() * b)
            } else {
                P::Diff::zero()
            }
        } else {
            let basis0 = knot_vector_u.bspline_basis_functions(degree0, m, u);
            let basis1 = knot_vector_v.bspline_basis_functions(degree1, n, v);
            self.combine_basis(&basis0, &basis1)
        }
    }
    #[inline(always)]
    fn evaluate(&self, u: f64, v: f64) -> P { P::from_vec(self.derivative_mn(0, 0, u, v)) }
    #[inline(always)]
    fn derivative_u(&self, u: f64, v: f64) -> P::Diff { self.derivative_mn(1, 0, u, v) }
    #[inline(always)]
    fn derivative_v(&self, u: f64, v: f64) -> P::Diff { self.derivative_mn(0, 1, u, v) }
    #[inline(always)]
    fn derivative_uu(&self, u: f64, v: f64) -> P::Diff { self.derivative_mn(2, 0, u, v) }
    #[inline(always)]
    fn derivative_uv(&self, u: f64, v: f64) -> P::Diff { self.derivative_mn(1, 1, u, v) }
    #[inline(always)]
    fn derivative_vv(&self, u: f64, v: f64) -> P::Diff { self.derivative_mn(0, 2, u, v) }

    #[inline(always)]
    fn parameter_range(&self) -> (ParameterRange, ParameterRange) { self.parameter_range() }
}
