use super::*;
use crate::errors::Error;
use std::iter::FusedIterator;

impl<P> BsplineSurface<P> {
    /// constructor.
    /// # Arguments
    /// * `knot_vecs` - the knot vectors
    /// * `control_points` - the vector of the control points
    /// # Panics
    /// There are 3 rules for construct B-spline curve.
    /// * The number of knots is less than or equal to the one of control points.
    /// * There exist at least two different knots.
    /// * There are at least one control point.
    #[inline(always)]
    pub fn new(
        knot_vecs: (KnotVector, KnotVector),
        control_points: Vec<Vec<P>>,
    ) -> BsplineSurface<P> {
        BsplineSurface::try_new(knot_vecs, control_points).unwrap_or_else(|e| panic!("{}", e))
    }

    /// constructor.
    /// # Arguments
    /// * `knot_vecs` - the knot vectors
    /// * `control_points` - the vector of the control points
    /// # Failures
    /// There are 3 rules for construct B-spline curve.
    /// * The number of knots is less than or equal to the one of control points.
    /// * There exist at least two different knots.
    /// * There are at least one control point.
    #[inline(always)]
    pub fn try_new(
        knot_vecs: (KnotVector, KnotVector),
        control_points: Vec<Vec<P>>,
    ) -> Result<BsplineSurface<P>> {
        if control_points.is_empty() || control_points[0].is_empty() {
            Err(Error::EmptyControlPoints)
        } else if knot_vecs.0.len() <= control_points.len() {
            Err(Error::TooShortKnotVector(
                knot_vecs.0.len(),
                control_points.len(),
            ))
        } else if knot_vecs.1.len() <= control_points[0].len() {
            Err(Error::TooShortKnotVector(
                knot_vecs.1.len(),
                control_points[0].len(),
            ))
        } else if knot_vecs.0.range_length().so_small() || knot_vecs.1.range_length().so_small() {
            Err(Error::ZeroRange)
        } else {
            let len = control_points[0].len();
            if control_points.iter().any(|vec| vec.len() != len) {
                Err(Error::IrregularControlPoints)
            } else {
                Ok(BsplineSurface::new_unchecked(knot_vecs, control_points))
            }
        }
    }

    /// constructor.
    /// # Arguments
    /// * `knot_vecs` - the knot vectors
    /// * `control_points` - the vector of the control points
    /// # Failures
    /// This method is prepared only for performance-critical development and is not recommended.
    /// This method does NOT check the 3 rules for constructing B-spline surface.
    /// The programmer must guarantee these conditions before using this method.
    #[inline(always)]
    pub const fn new_unchecked(
        knot_vecs: (KnotVector, KnotVector),
        control_points: Vec<Vec<P>>,
    ) -> BsplineSurface<P> {
        BsplineSurface {
            knot_vecs,
            control_points,
        }
    }

    /// constructor.
    /// # Arguments
    /// * `knot_vecs` - the knot vectors
    /// * `control_points` - the vector of the control points
    /// # Failures
    /// This method checks the 3 rules for constructing B-spline surface in the debug mode.
    /// The programmer must guarantee these conditions before using this method.
    #[inline(always)]
    pub fn debug_new(
        knot_vecs: (KnotVector, KnotVector),
        control_points: Vec<Vec<P>>,
    ) -> BsplineSurface<P> {
        match cfg!(debug_assertions) {
            true => Self::new(knot_vecs, control_points),
            false => Self::new_unchecked(knot_vecs, control_points),
        }
    }
    /// Returns the reference of the knot vectors.
    #[inline(always)]
    pub const fn knot_vectors(&self) -> &(KnotVector, KnotVector) { &self.knot_vecs }

    /// Renamed to [`knot_vectors`](Self::knot_vectors).
    #[deprecated(note = "renamed to knot_vectors")]
    #[inline(always)]
    pub const fn knot_vecs(&self) -> &(KnotVector, KnotVector) { &self.knot_vecs }

    /// Returns the u knot vector.
    #[inline(always)]
    pub const fn knot_vector_u(&self) -> &KnotVector { &self.knot_vecs.0 }
    /// Returns the v knot vector.
    #[inline(always)]
    pub const fn knot_vector_v(&self) -> &KnotVector { &self.knot_vecs.1 }

    /// Returns the `idx`th u knot.
    #[inline(always)]
    pub fn knot_u(&self, idx: usize) -> f64 { self.knot_vecs.0[idx] }
    /// returns the `idx`th v knot.
    #[inline(always)]
    pub fn knot_v(&self, idx: usize) -> f64 { self.knot_vecs.1[idx] }

    /// Returns the reference of the vector of the control points
    #[inline(always)]
    pub const fn control_points(&self) -> &Vec<Vec<P>> { &self.control_points }

    /// Returns the reference of the control point corresponding to the index `(idx0, idx1)`.
    #[inline(always)]
    pub fn control_point(&self, idx0: usize, idx1: usize) -> &P { &self.control_points[idx0][idx1] }
    /// Apply the given transformation to all control points.
    #[inline(always)]
    pub fn transform_control_points<F: FnMut(&mut P)>(&mut self, f: F) {
        self.control_points.iter_mut().flatten().for_each(f)
    }

    /// Returns the iterator over the control points in the `column_idx`th row.
    /// # Examples
    /// ```
    /// use monstertruck_geometry::prelude::*;
    /// let knot_vector_u = KnotVector::bezier_knot(1);
    /// let knot_vector_v = KnotVector::bezier_knot(2);
    /// let knot_vecs = (knot_vector_u, knot_vector_v);
    /// let control_points = vec![
    ///     vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 1.0), Vector3::new(2.0, 0.0, 2.0)],
    ///     vec![Vector3::new(0.0, 1.0, 0.0), Vector3::new(1.0, 1.0, 1.0), Vector3::new(2.0, 1.0, 2.0)],
    /// ];
    /// let bspsurface = BsplineSurface::new(knot_vecs, control_points);
    /// let mut iter = bspsurface.control_points_row_iter(1);
    /// assert_eq!(iter.next(), Some(&Vector3::new(1.0, 0.0, 1.0)));
    /// assert_eq!(iter.next(), Some(&Vector3::new(1.0, 1.0, 1.0)));
    /// assert_eq!(iter.next(), None);
    /// ```
    #[inline(always)]
    pub fn control_points_row_iter(
        &self,
        column_idx: usize,
    ) -> impl ExactSizeIterator<Item = &P> + FusedIterator<Item = &P> {
        self.control_points.iter().map(move |vec| &vec[column_idx])
    }

    /// Returns the iterator over the control points in the `row_idx`th row.
    /// # Examples
    /// ```
    /// use monstertruck_geometry::prelude::*;
    /// let knot_vector_u = KnotVector::bezier_knot(1);
    /// let knot_vector_v = KnotVector::bezier_knot(2);
    /// let knot_vecs = (knot_vector_u, knot_vector_v);
    /// let control_points = vec![
    ///     vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 1.0), Vector3::new(2.0, 0.0, 2.0)],
    ///     vec![Vector3::new(0.0, 1.0, 0.0), Vector3::new(1.0, 1.0, 1.0), Vector3::new(2.0, 1.0, 2.0)],
    /// ];
    /// let bspsurface = BsplineSurface::new(knot_vecs, control_points);
    /// let mut iter = bspsurface.control_points_column_iter(1);
    /// assert_eq!(iter.next(), Some(&Vector3::new(0.0, 1.0, 0.0)));
    /// assert_eq!(iter.next(), Some(&Vector3::new(1.0, 1.0, 1.0)));
    /// assert_eq!(iter.next(), Some(&Vector3::new(2.0, 1.0, 2.0)));
    /// assert_eq!(iter.next(), None);
    /// ```
    #[inline(always)]
    pub fn control_points_column_iter(&self, row_idx: usize) -> std::slice::Iter<'_, P> {
        self.control_points[row_idx].iter()
    }

    /// Returns the mutable reference of the control point corresponding to index `(idx0, idx1)`.
    #[inline(always)]
    pub fn control_point_mut(&mut self, idx0: usize, idx1: usize) -> &mut P {
        &mut self.control_points[idx0][idx1]
    }

    /// Returns the iterator on all control points
    #[inline(always)]
    pub fn control_points_mut(&mut self) -> impl Iterator<Item = &mut P> {
        self.control_points.iter_mut().flatten()
    }
    /// Returns the degrees of B-spline surface
    /// # Examples
    /// ```
    /// use monstertruck_geometry::prelude::*;
    /// let knot_vector_u = KnotVector::from(vec![0.0, 0.0, 1.0, 1.0]);
    /// let knot_vector_v = KnotVector::from(vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]);
    /// let knot_vecs = (knot_vector_u, knot_vector_v);
    /// let control_points = vec![
    ///     vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 1.0), Vector3::new(2.0, 0.0, 2.0)],
    ///     vec![Vector3::new(0.0, 1.0, 0.0), Vector3::new(1.0, 1.0, 1.0), Vector3::new(2.0, 1.0, 2.0)],
    /// ];
    /// let bspsurface = BsplineSurface::new(knot_vecs, control_points);
    /// assert_eq!(bspsurface.udegree(), 1);
    /// ```
    #[inline(always)]
    pub fn udegree(&self) -> usize { self.knot_vecs.0.len() - self.control_points.len() - 1 }

    /// Returns the degrees of B-spline surface
    /// # Examples
    /// ```
    /// use monstertruck_geometry::prelude::*;
    /// let knot_vector_u = KnotVector::from(vec![0.0, 0.0, 1.0, 1.0]);
    /// let knot_vector_v = KnotVector::from(vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]);
    /// let knot_vecs = (knot_vector_u, knot_vector_v);
    /// let control_points = vec![
    ///     vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 1.0), Vector3::new(2.0, 0.0, 2.0)],
    ///     vec![Vector3::new(0.0, 1.0, 0.0), Vector3::new(1.0, 1.0, 1.0), Vector3::new(2.0, 1.0, 2.0)],
    /// ];
    /// let bspsurface = BsplineSurface::new(knot_vecs, control_points);
    /// assert_eq!(bspsurface.vdegree(), 2);
    /// ```
    #[inline(always)]
    pub fn vdegree(&self) -> usize { self.knot_vecs.1.len() - self.control_points[0].len() - 1 }

    /// Returns the degrees of B-spline surface
    /// # Examples
    /// ```
    /// use monstertruck_geometry::prelude::*;
    /// let knot_vector_u = KnotVector::from(vec![0.0, 0.0, 1.0, 1.0]);
    /// let knot_vector_v = KnotVector::from(vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]);
    /// let knot_vecs = (knot_vector_u, knot_vector_v);
    /// let control_points = vec![
    ///     vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 1.0), Vector3::new(2.0, 0.0, 2.0)],
    ///     vec![Vector3::new(0.0, 1.0, 0.0), Vector3::new(1.0, 1.0, 1.0), Vector3::new(2.0, 1.0, 2.0)],
    /// ];
    /// let bspsurface = BsplineSurface::new(knot_vecs, control_points);
    /// assert_eq!(bspsurface.degrees(), (1, 2));
    /// ```
    #[inline(always)]
    pub fn degrees(&self) -> (usize, usize) { (self.udegree(), self.vdegree()) }
    /// Returns whether the knot vectors are clamped or not.
    #[inline(always)]
    pub fn is_clamped(&self) -> bool {
        self.knot_vecs.0.is_clamped(self.udegree()) && self.knot_vecs.1.is_clamped(self.vdegree())
    }

    /// Swaps two parameters.
    /// # Examples
    /// ```
    /// use monstertruck_geometry::prelude::*;
    /// let knot_vecs0 = (KnotVector::bezier_knot(1), KnotVector::bezier_knot(2));
    /// let control_points0 = vec![
    ///     vec![Vector2::new(0.0, 0.0), Vector2::new(0.5, -1.0), Vector2::new(1.0, 0.0)],
    ///     vec![Vector2::new(0.0, 1.0), Vector2::new(0.5, 2.0), Vector2::new(1.0, 1.0)],
    /// ];
    /// let mut bspsurface0 = BsplineSurface::new(knot_vecs0, control_points0);
    ///
    /// let knot_vecs1 = (KnotVector::bezier_knot(2), KnotVector::bezier_knot(1));
    /// let control_points1 = vec![
    ///     vec![Vector2::new(0.0, 0.0), Vector2::new(0.0, 1.0)],
    ///     vec![Vector2::new(0.5, -1.0), Vector2::new(0.5, 2.0)],
    ///     vec![Vector2::new(1.0, 0.0), Vector2::new(1.0, 1.0)],
    /// ];
    /// let mut bspsurface1 = BsplineSurface::new(knot_vecs1, control_points1);
    /// assert_eq!(bspsurface0.swap_axes(), &bspsurface1);
    /// ```
    pub fn swap_axes(&mut self) -> &mut Self
    where P: Clone {
        let knot_vec = self.knot_vecs.0.clone();
        self.knot_vecs.0 = self.knot_vecs.1.clone();
        self.knot_vecs.1 = knot_vec;

        // A degenerate (zero-row) control net has no rows to transpose and no
        // defined column count. This state is reachable via `cut_u`/`cut_v` when
        // the cut parameter `near()`-snaps to a domain-boundary knot (a zero-width
        // sub-range), so treat it as a total, graceful no-op beyond the knot swap
        // above rather than indexing `control_points[0]` (which panicked).
        if self.control_points.is_empty() {
            return self;
        }

        let n0 = self.control_points.len();
        let n1 = self.control_points[0].len();
        let mut new_points = vec![Vec::with_capacity(n0); n1];
        for pts in &self.control_points {
            for (vec0, pt) in new_points.iter_mut().zip(pts) {
                vec0.push(pt.clone());
            }
        }
        self.control_points = new_points;
        self
    }

    /// The range of the parameter of the surface.
    #[inline(always)]
    pub fn parameter_range(&self) -> (ParameterRange, ParameterRange) {
        // For B-splines, this is [knot[degree], knot[n_cv]] in each direction,
        // which is the valid evaluation domain.
        let udeg = self.knot_vecs.0.len() - self.control_points.len() - 1;
        let vdeg = self.knot_vecs.1.len() - self.control_points[0].len() - 1;
        (
            (
                Bound::Included(self.knot_vecs.0[udeg]),
                Bound::Included(self.knot_vecs.0[self.control_points.len()]),
            ),
            (
                Bound::Included(self.knot_vecs.1[vdeg]),
                Bound::Included(self.knot_vecs.1[self.control_points[0].len()]),
            ),
        )
    }
    /// Creates the curve whose control points are the `idx`th column control points of `self`.
    /// # Examples
    /// ```
    /// use monstertruck_geometry::prelude::*;
    /// let knot_vector_u = KnotVector::bezier_knot(1);
    /// let knot_vector_v = KnotVector::bezier_knot(2);
    /// let knot_vecs = (knot_vector_u, knot_vector_v);
    /// let control_points = vec![
    ///     vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 1.0), Vector3::new(2.0, 0.0, 2.0)],
    ///     vec![Vector3::new(0.0, 1.0, 0.0), Vector3::new(1.0, 1.0, 1.0), Vector3::new(2.0, 1.0, 2.0)],
    /// ];
    /// let bspsurface = BsplineSurface::new(knot_vecs, control_points);
    /// let bspcurve = bspsurface.curve_v(1);
    ///
    /// assert_eq!(bspcurve.knot_vector(), &KnotVector::bezier_knot(2));
    /// assert_eq!(
    ///     bspcurve.control_points(),
    ///     &vec![Vector3::new(0.0, 1.0, 0.0), Vector3::new(1.0, 1.0, 1.0), Vector3::new(2.0, 1.0, 2.0)],
    /// );
    /// ```
    pub fn curve_v(&self, index_u: usize) -> BsplineCurve<P>
    where P: Clone {
        let knot_vec = self.knot_vector_v().clone();
        let control_points = self.control_points[index_u].clone();
        BsplineCurve::new_unchecked(knot_vec, control_points)
    }
    /// Deprecated alias for [`curve_v`](BsplineSurface::curve_v), the name
    /// upstream `truck` uses.
    ///
    /// Renamed because the row/column vocabulary is crossed with respect to the
    /// parameter that actually varies: this method fixes the u-index and varies
    /// **v**, so `curve_v` says what you get and `index_u` says what you pin.
    #[deprecated(since = "0.3.4", note = "renamed to `curve_v` (it varies v)")]
    #[inline(always)]
    pub fn column_curve(&self, row_idx: usize) -> BsplineCurve<P>
    where P: Clone {
        self.curve_v(row_idx)
    }
    /// Creates the sectional curve along u, at the given v control-point index.
    /// # Examples
    /// ```
    /// use monstertruck_geometry::prelude::*;
    /// let knot_vector_u = KnotVector::bezier_knot(1);
    /// let knot_vector_v = KnotVector::bezier_knot(2);
    /// let knot_vecs = (knot_vector_u, knot_vector_v);
    /// let control_points = vec![
    ///     vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 1.0), Vector3::new(2.0, 0.0, 2.0)],
    ///     vec![Vector3::new(0.0, 1.0, 0.0), Vector3::new(1.0, 1.0, 1.0), Vector3::new(2.0, 1.0, 2.0)],
    /// ];
    /// let bspsurface = BsplineSurface::new(knot_vecs, control_points);
    /// let bspcurve = bspsurface.curve_u(1);
    ///
    /// assert_eq!(bspcurve.knot_vector(), &KnotVector::bezier_knot(1));
    /// assert_eq!(
    ///     bspcurve.control_points(),
    ///     &vec![Vector3::new(1.0, 0.0, 1.0), Vector3::new(1.0, 1.0, 1.0)],
    /// );
    /// ```
    pub fn curve_u(&self, index_v: usize) -> BsplineCurve<P>
    where P: Clone {
        let knot_vec = self.knot_vector_u().clone();
        let control_points: Vec<_> = self.control_points_row_iter(index_v).cloned().collect();
        BsplineCurve::new_unchecked(knot_vec, control_points)
    }
    /// Deprecated alias for [`curve_u`](BsplineSurface::curve_u), the name
    /// upstream `truck` uses.
    ///
    /// Renamed because the row/column vocabulary is crossed with respect to the
    /// parameter that actually varies: this method fixes the v-index and varies
    /// **u**, so `curve_u` says what you get and `index_v` says what you pin.
    #[deprecated(since = "0.3.4", note = "renamed to `curve_u` (it varies u)")]
    #[inline(always)]
    pub fn row_curve(&self, column_idx: usize) -> BsplineCurve<P>
    where P: Clone {
        self.curve_u(column_idx)
    }
}

impl<V: Homogeneous> BsplineSurface<V> {
    /// lift up control points to homogeneous coordinate.
    pub fn lift_up(surface: BsplineSurface<V::Point>) -> Self {
        let control_points = surface
            .control_points
            .into_iter()
            .map(|vec| vec.into_iter().map(V::from_point).collect())
            .collect();
        BsplineSurface::new_unchecked(surface.knot_vecs, control_points)
    }
}

impl<V: Tolerance> BsplineSurface<V> {
    /// Returns whether all control points are same or not.
    /// If the knot vector is clamped, it means whether the curve is constant or not.
    /// # Examples
    /// ```
    /// use monstertruck_geometry::prelude::*;
    /// let knot_vector_u = KnotVector::bezier_knot(1);
    /// let knot_vector_v = KnotVector::bezier_knot(2);
    /// let pt = Vector2::new(1.0, 2.0);
    /// let control_points = vec![
    ///     vec![pt.clone(), pt.clone(), pt.clone()],
    ///     vec![pt.clone(), pt.clone(), pt.clone()],
    /// ];
    /// let mut bspsurface = BsplineSurface::new((knot_vector_u, knot_vector_v), control_points);
    /// assert!(bspsurface.is_const());
    ///
    /// *bspsurface.control_point_mut(1, 2) = Vector2::new(2.0, 3.0);
    /// assert!(!bspsurface.is_const());
    /// ```
    /// # Remarks
    /// If the knot vector is not clamped and the Bspline basis function is not partition of unity,
    /// then perhaps returns true even if the surface is not constant.
    /// ```
    /// use monstertruck_geometry::prelude::*;
    /// let knot_vector_u = KnotVector::uniform_knot(1, 5);
    /// let knot_vector_v = KnotVector::uniform_knot(1, 5);
    /// let pt = Vector2::new(1.0, 2.0);
    /// let control_points = vec![
    ///     vec![pt.clone(), pt.clone(), pt.clone()],
    ///     vec![pt.clone(), pt.clone(), pt.clone()],
    /// ];
    /// let mut bspsurface = BsplineSurface::new((knot_vector_u, knot_vector_v), control_points);
    ///
    /// // bspsurface is not constant.
    /// assert_eq!(bspsurface.subs(0.0, 0.0), Vector2::new(0.0, 0.0));
    /// assert_ne!(bspsurface.subs(0.5, 0.5), Vector2::new(0.0, 0.0));
    ///
    /// // bspsurface.is_const() is true.
    /// assert!(bspsurface.is_const());
    /// ```
    #[inline(always)]
    pub fn is_const(&self) -> bool {
        for vec in self.control_points.iter().flat_map(|pts| pts.iter()) {
            if !vec.near(&self.control_points[0][0]) {
                return false;
            }
        }
        true
    }
}

impl<V: Bounded> BsplineSurface<V> {
    /// Returns the bounding box including all control points.
    #[inline(always)]
    pub fn roughly_bounding_box(&self) -> BoundingBox<V> {
        self.control_points.iter().flatten().collect()
    }
}
