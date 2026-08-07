use super::*;
use crate::errors::Error;

impl<P: ControlPoint<f64> + Tolerance> BsplineSurface<P> {
    /// Adds a knot `x` of the first parameter `u`, and do not change `self` as a surface.
    /// # Examples
    /// ```
    /// use monstertruck_geometry::prelude::*;
    /// let knot_vecs = (KnotVector::bezier_knot(1), KnotVector::bezier_knot(2));
    /// let control_points = vec![
    ///     vec![Vector2::new(0.0, 0.0), Vector2::new(0.5, -1.0), Vector2::new(1.0, 0.0)],
    ///     vec![Vector2::new(0.0, 1.0), Vector2::new(0.5, 2.0), Vector2::new(1.0, 1.0)],
    /// ];
    /// let mut bspsurface = BsplineSurface::new(knot_vecs, control_points);
    /// let org_surface = bspsurface.clone();
    /// bspsurface.add_knot_u(0.0).add_knot_u(0.3).add_knot_u(0.5).add_knot_u(1.0);
    /// assert!(bspsurface.near2_as_surface(&org_surface));
    /// assert_eq!(bspsurface.knot_vector_u().len(), org_surface.knot_vector_u().len() + 4);
    /// ```
    pub fn add_knot_u(&mut self, x: f64) -> &mut Self {
        let k = self.udegree();
        let n0 = self.control_points.len();
        let n1 = self.control_points[0].len();
        let knot_vector_u = &mut self.knot_vecs.0;
        let control_points = &mut self.control_points;
        if x < knot_vector_u[0] {
            knot_vector_u.add_knot(x);
            control_points.insert(0, vec![P::origin(); n1]);
            return self;
        }

        let idx = knot_vector_u.add_knot(x);
        let start = idx.saturating_sub(k);
        let end = if idx > n0 {
            control_points.push(vec![P::origin(); n1]);
            n0 + 1
        } else {
            control_points.insert(idx - 1, control_points[idx - 1].clone());
            idx
        };
        for i in start..end {
            let i0 = end + start - i - 1;
            let delta = self.knot_u(i0 + k + 1) - self.knot_u(i0);
            let a = inv_or_zero(delta) * (self.knot_u(idx) - self.knot_u(i0));
            for j in 0..n1 {
                let p = self.udelta_control_points(i0, j) * (1.0 - a);
                self.control_points[i0][j] -= p;
            }
        }
        self
    }

    /// Adds a knot `x` for the second parameter, and do not change `self` as a surface.
    /// Return `false` if cannot add the knot, i.e.
    /// * the index of `x` will be lower than the degree, or
    /// * the index of `x` will be higher than the number of control points.
    /// # Examples
    /// ```
    /// use monstertruck_geometry::prelude::*;
    /// let knot_vecs = (KnotVector::bezier_knot(1), KnotVector::bezier_knot(2));
    /// let control_points = vec![
    ///     vec![Vector2::new(0.0, 0.0), Vector2::new(0.5, -1.0), Vector2::new(1.0, 0.0)],
    ///     vec![Vector2::new(0.0, 1.0), Vector2::new(0.5, 2.0), Vector2::new(1.0, 1.0)],
    /// ];
    /// let mut bspsurface = BsplineSurface::new(knot_vecs, control_points);
    /// let org_surface = bspsurface.clone();
    /// bspsurface.add_knot_v(0.0).add_knot_v(0.3).add_knot_v(0.5).add_knot_v(1.0);
    /// assert!(bspsurface.near2_as_surface(&org_surface));
    /// assert_eq!(bspsurface.knot_vector_v().len(), org_surface.knot_vector_v().len() + 4);
    /// ```
    pub fn add_knot_v(&mut self, x: f64) -> &mut Self {
        if x < self.knot_vecs.1[0] {
            self.knot_vecs.1.add_knot(x);
            self.control_points
                .iter_mut()
                .for_each(|vec| vec.insert(0, P::origin()));
            return self;
        }

        let k = self.vdegree();
        let n0 = self.control_points.len();
        let n1 = self.control_points[0].len();

        let idx = self.knot_vecs.1.add_knot(x);
        let start = idx.saturating_sub(k);
        let end = if idx > n1 {
            self.control_points
                .iter_mut()
                .for_each(|vec| vec.push(P::origin()));
            n1 + 1
        } else {
            self.control_points
                .iter_mut()
                .for_each(|vec| vec.insert(idx - 1, vec[idx - 1]));
            idx
        };
        for j in start..end {
            let j0 = end + start - j - 1;
            let delta = self.knot_vecs.1[j0 + k + 1] - self.knot_vecs.1[j0];
            let a = inv_or_zero(delta) * (self.knot_vecs.1[idx] - self.knot_vecs.1[j0]);
            for i in 0..n0 {
                let p = self.vdelta_control_points(i, j0) * (1.0 - a);
                self.control_points[i][j0] -= p;
            }
        }
        self
    }

    /// Removes the knot_u corresponding to the indice `idx`, and do not change `self` as a curve.
    /// If the knot cannot be removed, returns
    /// [`Error::CannotRemoveKnot`](./errors/enum.Error.html#variant.CannotRemoveKnot).
    /// # Examples
    /// ```
    /// use monstertruck_geometry::prelude::*;
    /// use monstertruck_geometry::errors::Error;
    /// let knot_vecs = (KnotVector::bezier_knot(2), KnotVector::bezier_knot(2));
    /// let control_points = vec![
    ///     vec![Vector2::new(0.0, 0.0), Vector2::new(0.5, -1.0), Vector2::new(1.0, 0.0)],
    ///     vec![Vector2::new(0.0, 1.0), Vector2::new(0.5, 2.0), Vector2::new(1.0, 1.0)],
    ///     vec![Vector2::new(0.0, 2.0), Vector2::new(0.5, -1.0), Vector2::new(1.0, 2.0)],
    /// ];
    /// let mut bspsurface = BsplineSurface::new(knot_vecs, control_points);
    /// let org_surface = bspsurface.clone();
    ///
    /// bspsurface.add_knot_u(0.3).add_knot_u(0.5);
    ///
    /// assert!(bspsurface.try_remove_knot_u(3).is_ok());
    /// assert_eq!(bspsurface.try_remove_knot_u(2), Err(Error::CannotRemoveKnot(2)));
    ///
    /// assert_eq!(bspsurface.knot_vector_u().len(), org_surface.knot_vector_u().len() + 1);
    /// assert!(bspsurface.near2_as_surface(&org_surface));
    /// ```
    pub fn try_remove_knot_u(&mut self, idx: usize) -> Result<&mut Self> {
        let k = self.udegree();
        let knot_vec = self.knot_vector_u();
        let n = self.control_points.len();

        if idx < k + 1 || idx >= n {
            return Err(Error::CannotRemoveKnot(idx));
        }

        let mut new_points = Vec::with_capacity(k + 1);
        let first_vec = self
            .control_points_column_iter(idx - k - 1)
            .cloned()
            .collect::<Vec<_>>();
        new_points.push(first_vec);
        for i in (idx - k)..idx {
            let delta = knot_vec[i + k + 1] - knot_vec[i];
            let a = inv_or_zero(delta) * (knot_vec[idx] - knot_vec[i]);
            if a.so_small() {
                break;
            } else {
                // SAFETY: `new_points` was seeded with one element and only grows.
                let vec = self
                    .control_points_column_iter(i)
                    .zip(new_points.last().unwrap())
                    .map(|(pt0, pt1)| *pt1 + (*pt0 - *pt1) / a)
                    .collect();
                new_points.push(vec);
            }
        }

        // SAFETY: `new_points` was seeded with one element and only grows.
        for (pt0, pt1) in self
            .control_points_column_iter(idx)
            .zip(new_points.last().unwrap())
        {
            if !pt0.near(pt1) {
                return Err(Error::CannotRemoveKnot(idx));
            }
        }

        for (i, vec) in new_points.into_iter().skip(1).enumerate() {
            self.control_points[idx - k + i] = vec;
        }

        self.control_points.remove(idx);
        self.knot_vecs.0.remove(idx);
        Ok(self)
    }

    /// Removes the knot_u corresponding to the indices `idx`, and do not change `self` as a curve.
    /// If cannot remove the knot, do not change `self` and return `self`.
    /// # Examples
    /// ```
    /// use monstertruck_geometry::prelude::*;
    /// use monstertruck_geometry::errors::Error;
    /// let knot_vecs = (KnotVector::bezier_knot(2), KnotVector::bezier_knot(2));
    /// let control_points = vec![
    ///     vec![Vector2::new(0.0, 0.0), Vector2::new(0.5, -1.0), Vector2::new(1.0, 0.0)],
    ///     vec![Vector2::new(0.0, 1.0), Vector2::new(0.5, 2.0), Vector2::new(1.0, 1.0)],
    ///     vec![Vector2::new(0.0, 2.0), Vector2::new(0.5, -1.0), Vector2::new(1.0, 2.0)],
    /// ];
    /// let mut bspsurface = BsplineSurface::new(knot_vecs, control_points);
    /// let org_surface = bspsurface.clone();
    ///
    /// bspsurface.add_knot_u(0.3).add_knot_u(0.5);
    /// bspsurface.remove_knot_u(3).remove_knot_u(3);
    ///
    /// assert!(bspsurface.near2_as_surface(&org_surface));
    /// assert_eq!(bspsurface.knot_vector_u().len(), org_surface.knot_vector_u().len())
    /// ```
    #[inline(always)]
    pub fn remove_knot_u(&mut self, idx: usize) -> &mut Self {
        let _ = self.try_remove_knot_u(idx);
        self
    }

    /// Removes a knot_v corresponding to the indice `idx`, and do not change `self` as a curve.
    /// If the knot cannot be removed, returns
    /// [`Error::CannotRemoveKnot`](./errors/enum.Error.html#variant.CannotRemoveKnot).
    /// # Examples
    /// ```
    /// use monstertruck_geometry::prelude::*;
    /// use monstertruck_geometry::errors::Error;
    /// let knot_vecs = (KnotVector::bezier_knot(2), KnotVector::bezier_knot(2));
    /// let control_points = vec![
    ///     vec![Vector2::new(0.0, 0.0), Vector2::new(0.5, -1.0), Vector2::new(1.0, 0.0)],
    ///     vec![Vector2::new(0.0, 1.0), Vector2::new(0.5, 2.0), Vector2::new(1.0, 1.0)],
    ///     vec![Vector2::new(0.0, 2.0), Vector2::new(0.5, -1.0), Vector2::new(1.0, 2.0)],
    /// ];
    /// let mut bspsurface = BsplineSurface::new(knot_vecs, control_points);
    /// let org_surface = bspsurface.clone();
    ///
    /// bspsurface.add_knot_v(0.3).add_knot_v(0.5);
    /// assert!(bspsurface.try_remove_knot_v(3).is_ok());
    /// assert_eq!(bspsurface.try_remove_knot_v(2), Err(Error::CannotRemoveKnot(2)));
    ///
    /// assert!(bspsurface.near2_as_surface(&org_surface));
    /// assert_eq!(bspsurface.knot_vector_v().len(), org_surface.knot_vector_v().len() + 1);
    /// ```
    pub fn try_remove_knot_v(&mut self, idx: usize) -> Result<&mut Self> {
        let (_, k) = self.degrees();
        let knot_vec = self.knot_vector_v();
        let n = self.control_points[0].len();

        if idx < k + 1 || idx >= n {
            return Err(Error::CannotRemoveKnot(idx));
        }

        let mut new_points = Vec::with_capacity(k + 1);
        let first_vec = self
            .control_points_row_iter(idx - k - 1)
            .cloned()
            .collect::<Vec<_>>();
        new_points.push(first_vec);
        for i in (idx - k)..idx {
            let delta = knot_vec[i + k + 1] - knot_vec[i];
            let a = inv_or_zero(delta) * (knot_vec[idx] - knot_vec[i]);
            if a.so_small() {
                break;
            } else {
                // SAFETY: `new_points` was seeded with one element and only grows.
                let vec = self
                    .control_points_row_iter(i)
                    .zip(new_points.last().unwrap())
                    .map(|(pt0, pt1)| *pt1 + (*pt0 - *pt1) / a)
                    .collect();
                new_points.push(vec);
            }
        }

        // SAFETY: `new_points` was seeded with one element and only grows.
        for (pt0, pt1) in self
            .control_points_row_iter(idx)
            .zip(new_points.last().unwrap())
        {
            if !pt0.near(pt1) {
                return Err(Error::CannotRemoveKnot(idx));
            }
        }

        for (i, vec) in new_points.into_iter().skip(1).enumerate() {
            for (j, pt) in vec.into_iter().enumerate() {
                self.control_points[j][idx - k + i] = pt;
            }
        }

        for vec in &mut self.control_points {
            vec.remove(idx);
        }
        self.knot_vecs.1.remove(idx);
        Ok(self)
    }

    /// Removes a knot_u corresponding to the indices `idx`, and do not change `self` as a curve.
    /// If cannot remove the knot, do not change `self` and return `self`.
    /// # Examples
    /// ```
    /// use monstertruck_geometry::prelude::*;
    /// use monstertruck_geometry::errors::Error;
    /// let knot_vecs = (KnotVector::bezier_knot(2), KnotVector::bezier_knot(2));
    /// let control_points = vec![
    ///     vec![Vector2::new(0.0, 0.0), Vector2::new(0.5, -1.0), Vector2::new(1.0, 0.0)],
    ///     vec![Vector2::new(0.0, 1.0), Vector2::new(0.5, 2.0), Vector2::new(1.0, 1.0)],
    ///     vec![Vector2::new(0.0, 2.0), Vector2::new(0.5, -1.0), Vector2::new(1.0, 2.0)],
    /// ];
    /// let mut bspsurface = BsplineSurface::new(knot_vecs, control_points);
    /// let org_surface = bspsurface.clone();
    ///
    /// bspsurface.add_knot_v(0.3).add_knot_v(0.5);
    /// bspsurface.remove_knot_v(3).remove_knot_v(3);
    ///
    /// assert!(bspsurface.near2_as_surface(&org_surface));
    /// assert_eq!(bspsurface.knot_vector_v().len(), org_surface.knot_vector_v().len())
    /// ```
    #[inline(always)]
    pub fn remove_knot_v(&mut self, idx: usize) -> &mut Self {
        let _ = self.try_remove_knot_v(idx);
        self
    }

    /// Elevates the vdegree.
    /// # Examples
    /// ```
    /// use monstertruck_geometry::prelude::*;
    /// let knot_vecs = (KnotVector::bezier_knot(2), KnotVector::bezier_knot(2));
    /// let control_points = vec![
    ///     vec![Vector2::new(0.0, 0.0), Vector2::new(0.5, -1.0), Vector2::new(1.0, 0.0)],
    ///     vec![Vector2::new(0.0, 1.0), Vector2::new(0.5, 2.0), Vector2::new(1.0, 1.0)],
    ///     vec![Vector2::new(0.0, 2.0), Vector2::new(0.5, -1.0), Vector2::new(1.0, 2.0)],
    /// ];
    /// let mut bspsurface = BsplineSurface::new(knot_vecs, control_points);
    /// let org_surface = bspsurface.clone();
    ///
    /// bspsurface.elevate_vdegree();
    ///
    /// assert_eq!(bspsurface.udegree(), org_surface.udegree());
    /// assert_eq!(bspsurface.vdegree(), org_surface.vdegree() + 1);
    /// assert!(bspsurface.near2_as_surface(&org_surface));
    /// ```
    pub fn elevate_vdegree(&mut self) -> &mut Self {
        let mut new_knot_vec = KnotVector::new();
        for (i, vec) in self.control_points.iter_mut().enumerate() {
            let knot_vec = self.knot_vecs.1.clone();
            let control_points = vec.clone();
            let mut curve = BsplineCurve::new(knot_vec, control_points);
            curve.elevate_degree();
            if i == 0 {
                new_knot_vec = curve.knot_vector().clone();
            }
            *vec = curve.control_points;
        }
        self.knot_vecs.1 = new_knot_vec;
        self
    }

    /// Elevates the udegree.
    /// # Examples
    /// ```
    /// use monstertruck_geometry::prelude::*;
    /// let knot_vecs = (KnotVector::bezier_knot(2), KnotVector::bezier_knot(2));
    /// let control_points = vec![
    ///     vec![Vector2::new(0.0, 0.0), Vector2::new(0.5, -1.0), Vector2::new(1.0, 0.0)],
    ///     vec![Vector2::new(0.0, 1.0), Vector2::new(0.5, 2.0), Vector2::new(1.0, 1.0)],
    ///     vec![Vector2::new(0.0, 2.0), Vector2::new(0.5, -1.0), Vector2::new(1.0, 2.0)],
    /// ];
    /// let mut bspsurface = BsplineSurface::new(knot_vecs, control_points);
    /// let org_surface = bspsurface.clone();
    ///
    /// bspsurface.elevate_udegree();
    ///
    /// assert_eq!(bspsurface.udegree(), org_surface.udegree() + 1);
    /// assert_eq!(bspsurface.vdegree(), org_surface.vdegree());
    /// assert!(bspsurface.near2_as_surface(&org_surface));
    /// ```
    pub fn elevate_udegree(&mut self) -> &mut Self {
        self.swap_axes();
        self.elevate_vdegree();
        self.swap_axes();
        self
    }

    /// Aligns the udegree with the same degrees.
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
    /// let mut bspsurface = BsplineSurface::new(knot_vecs, control_points);
    /// let org_surface = bspsurface.clone();
    ///
    /// assert_ne!(bspsurface.udegree(), bspsurface.vdegree());
    /// bspsurface.syncro_uvdegrees();
    /// assert_eq!(bspsurface.udegree(), bspsurface.vdegree());
    /// assert!(bspsurface.near2_as_surface(&org_surface));
    /// ```
    pub fn syncro_uvdegrees(&mut self) -> &mut Self {
        if self.udegree() > self.vdegree() {
            for _ in 0..(self.udegree() - self.vdegree()) {
                self.elevate_vdegree();
            }
        }
        if self.vdegree() > self.udegree() {
            for _ in 0..(self.vdegree() - self.udegree()) {
                self.elevate_udegree();
            }
        }
        self
    }

    /// Makes the knot_u vector and the knot_v vector the same knot vector.
    /// # Examples
    /// ```
    /// use monstertruck_geometry::prelude::*;
    /// let knot_vector_u = KnotVector::uniform_knot(1, 2);
    /// let knot_vector_v = KnotVector::bezier_knot(2);
    /// let knot_vecs = (knot_vector_u, knot_vector_v);
    /// let control_points = vec![
    ///     vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 1.0), Vector3::new(2.0, 0.0, 2.0)],
    ///     vec![Vector3::new(0.0, 1.0, 0.0), Vector3::new(1.0, 1.0, 1.0), Vector3::new(2.0, 1.0, 2.0)],
    ///     vec![Vector3::new(0.0, 2.0, 0.0), Vector3::new(1.0, 2.0, 1.0), Vector3::new(2.0, 2.0, 2.0)],
    /// ];
    /// let mut bspsurface = BsplineSurface::new(knot_vecs, control_points);
    /// let org_surface = bspsurface.clone();
    ///
    /// assert_ne!(bspsurface.knot_vector_u(), bspsurface.knot_vector_v());
    /// bspsurface.syncro_uvknots();
    /// assert_eq!(bspsurface.knot_vector_u(), bspsurface.knot_vector_v());
    /// assert!(bspsurface.near2_as_surface(&org_surface));
    /// ```
    pub fn syncro_uvknots(&mut self) -> &mut Self {
        self.knot_vecs.0.normalize();
        self.knot_vecs.1.normalize();
        let mut i = 0;
        let mut j = 0;
        while !self.knot_u(i).near2(&1.0) || !self.knot_v(j).near2(&1.0) {
            if self.knot_u(i) - self.knot_v(j) > TOLERANCE {
                self.add_knot_u(self.knot_v(j));
            } else if self.knot_v(j) - self.knot_u(i) > TOLERANCE {
                self.add_knot_v(self.knot_u(i));
            }
            i += 1;
            j += 1;
        }

        let ulen = self.knot_vector_u().len();
        let vlen = self.knot_vector_v().len();
        use std::cmp::Ordering;
        match usize::cmp(&ulen, &vlen) {
            Ordering::Less => {
                for _ in 0..vlen - ulen {
                    self.add_knot_u(1.0);
                }
            }
            Ordering::Greater => {
                for _ in 0..ulen - vlen {
                    self.add_knot_v(1.0);
                }
            }
            _ => {}
        }
        self
    }

    /// Normalizes the knot vectors
    #[inline(always)]
    pub fn knot_normalize(&mut self) -> &mut Self {
        self.knot_vecs.0.normalize();
        self.knot_vecs.1.normalize();
        self
    }

    /// Translates the knot vectors.
    #[inline(always)]
    pub fn knot_translate(&mut self, x: f64, y: f64) -> &mut Self {
        self.knot_vecs.0.translate(x);
        self.knot_vecs.1.translate(y);
        self
    }

    /// Removes knots in order from the back
    pub fn optimize(&mut self) -> &mut Self {
        loop {
            let (n0, n1) = (self.knot_vecs.0.len(), self.knot_vecs.1.len());
            let mut flag = true;
            for i in 1..=n0 {
                flag = flag && self.try_remove_knot_u(n0 - i).is_err();
            }
            for j in 1..=n1 {
                flag = flag && self.try_remove_knot_v(n1 - j).is_err();
            }
            if flag {
                break;
            }
        }
        self
    }
}
