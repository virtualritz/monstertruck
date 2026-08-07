use super::*;

impl<P: ControlPoint<f64> + Tolerance> BsplineSurface<P> {
    /// Creates a surface with normailized knot vectors connecting two curves.
    /// # Examples
    /// ```
    /// use monstertruck_geometry::prelude::*;
    /// let knot_vec0 = KnotVector::bezier_knot(2);
    /// let control_points0 = vec![Vector2::new(0.0, 0.0), Vector2::new(0.5, -1.0), Vector2::new(1.0, 0.0)];
    /// let bspcurve0 = BsplineCurve::new(knot_vec0, control_points0);
    ///
    /// let knot_vec1 = KnotVector::bezier_knot(2);
    /// let control_points1 = vec![Vector2::new(0.0, 2.0), Vector2::new(0.5, 1.0), Vector2::new(1.0, 2.0)];
    /// let bspcurve1 = BsplineCurve::new(knot_vec1, control_points1);
    ///
    /// let homotopy_surface = BsplineSurface::homotopy(bspcurve0, bspcurve1);
    /// assert_eq!(
    ///     homotopy_surface.control_points(),
    ///     &vec![
    ///         vec![Vector2::new(0.0, 0.0), Vector2::new(0.0, 2.0)],
    ///         vec![Vector2::new(0.5, -1.0), Vector2::new(0.5, 1.0)],
    ///         vec![Vector2::new(1.0, 0.0), Vector2::new(1.0, 2.0)],
    ///     ],
    /// );
    /// ```
    pub fn homotopy(
        mut bspcurve0: BsplineCurve<P>,
        mut bspcurve1: BsplineCurve<P>,
    ) -> BsplineSurface<P> {
        bspcurve0.syncro_degree(&mut bspcurve1);

        //bspcurve0.optimize();
        //bspcurve1.optimize();

        bspcurve0.syncro_knots(&mut bspcurve1);

        let knot_vector_u = bspcurve0.knot_vector().clone();
        let knot_vector_v = KnotVector::from(vec![0.0, 0.0, 1.0, 1.0]);
        let control_points: Vec<Vec<_>> = (0..bspcurve0.control_points().len())
            .map(|i| vec![*bspcurve0.control_point(i), *bspcurve1.control_point(i)])
            .collect();
        BsplineSurface::new_unchecked((knot_vector_u, knot_vector_v), control_points)
    }

    /// Creates a skinned (lofted) surface through N section curves.
    ///
    /// Generalizes [`homotopy`](Self::homotopy) to an arbitrary number of sections.
    /// The u-direction follows each section curve; the v-direction linearly interpolates
    /// between sections with a degree-1 knot vector.
    ///
    /// All input curves are made compatible (same degree and knot vector) before
    /// surface construction. The curve shapes are preserved.
    ///
    /// # Panics
    ///
    /// Panics if `curves` is empty.
    ///
    /// # Examples
    ///
    /// ```
    /// use monstertruck_geometry::prelude::*;
    ///
    /// let c0 = BsplineCurve::new(
    ///     KnotVector::bezier_knot(2),
    ///     vec![Vector2::new(0.0, 0.0), Vector2::new(0.5, -1.0), Vector2::new(1.0, 0.0)],
    /// );
    /// let c1 = BsplineCurve::new(
    ///     KnotVector::bezier_knot(2),
    ///     vec![Vector2::new(0.0, 1.0), Vector2::new(0.5, 0.5), Vector2::new(1.0, 1.0)],
    /// );
    /// let c2 = BsplineCurve::new(
    ///     KnotVector::bezier_knot(2),
    ///     vec![Vector2::new(0.0, 2.0), Vector2::new(0.5, 1.0), Vector2::new(1.0, 2.0)],
    /// );
    /// let surface = BsplineSurface::skin(vec![c0, c1, c2]);
    /// // At v=0, v=0.5, v=1 the surface reproduces the three section curves.
    /// // Endpoints are exact.
    /// assert_near2!(surface.subs(0.0, 0.0), Vector2::new(0.0, 0.0));
    /// assert_near2!(surface.subs(1.0, 0.0), Vector2::new(1.0, 0.0));
    /// assert_near2!(surface.subs(0.0, 0.5), Vector2::new(0.0, 1.0));
    /// assert_near2!(surface.subs(1.0, 0.5), Vector2::new(1.0, 1.0));
    /// assert_near2!(surface.subs(0.0, 1.0), Vector2::new(0.0, 2.0));
    /// assert_near2!(surface.subs(1.0, 1.0), Vector2::new(1.0, 2.0));
    /// ```
    pub fn skin(mut curves: Vec<BsplineCurve<P>>) -> BsplineSurface<P> {
        assert!(
            !curves.is_empty(),
            "skin requires at least one section curve"
        );
        if curves.len() == 1 {
            // Degenerate: single curve -> constant surface in v.
            let c = &mut curves[0];
            c.knot_normalize();
            let knot_vector_u = c.knot_vector().clone();
            let knot_vector_v = KnotVector::from(vec![0.0, 0.0, 1.0, 1.0]);
            let control_points: Vec<Vec<_>> =
                c.control_points().iter().map(|p| vec![*p, *p]).collect();
            return BsplineSurface::new_unchecked((knot_vector_u, knot_vector_v), control_points);
        }
        if curves.len() == 2 {
            return Self::homotopy(curves.swap_remove(0), curves.swap_remove(0));
        }

        // Make all section curves compatible.
        compat::make_curves_compatible(&mut curves)
            .expect("skin: compatibility normalization failed on non-empty curve set");

        let n = curves.len();
        let m = curves[0].control_points().len();

        // Build the v-direction knot vector (degree 1, clamped, uniform).
        let mut v_knots = Vec::with_capacity(n + 2);
        v_knots.push(0.0);
        (0..n).for_each(|i| v_knots.push(i as f64 / (n - 1) as f64));
        v_knots.push(1.0);
        let knot_vector_v = KnotVector::from(v_knots);

        let knot_vector_u = curves[0].knot_vector().clone();
        let control_points: Vec<Vec<P>> = (0..m)
            .map(|j| curves.iter().map(|c| *c.control_point(j)).collect())
            .collect();
        BsplineSurface::new_unchecked((knot_vector_u, knot_vector_v), control_points)
    }
}

impl BsplineSurface<Point3> {
    /// Sweeps a profile curve along a rail curve with tangent-alignment framing.
    ///
    /// The profile is assumed to lie in a plane perpendicular to the rail's
    /// tangent at the start. At each of `n_sections` sample points along the
    /// rail, the profile is rotated to align with the local tangent and
    /// translated to the rail point. The resulting sections are then skinned.
    ///
    /// # Panics
    ///
    /// Panics if `n_sections < 2`.
    ///
    /// # Examples
    ///
    /// ```
    /// use monstertruck_geometry::prelude::*;
    ///
    /// // Sweep a small circle profile along a straight rail (should approximate extrusion).
    /// let rail = BsplineCurve::new(
    ///     KnotVector::bezier_knot(1),
    ///     vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 5.0)],
    /// );
    /// let profile = BsplineCurve::new(
    ///     KnotVector::bezier_knot(1),
    ///     vec![Point3::new(-1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
    /// );
    /// let surface = BsplineSurface::sweep_rail(profile, &rail, 3);
    /// // At v=0 (rail start), the surface reproduces the profile.
    /// assert_near2!(surface.subs(0.0, 0.0), Point3::new(-1.0, 0.0, 0.0));
    /// assert_near2!(surface.subs(1.0, 0.0), Point3::new(1.0, 0.0, 0.0));
    /// // At v=1 (rail end), the profile is translated to the rail endpoint.
    /// assert_near2!(surface.subs(0.0, 1.0), Point3::new(-1.0, 0.0, 5.0));
    /// assert_near2!(surface.subs(1.0, 1.0), Point3::new(1.0, 0.0, 5.0));
    /// ```
    pub fn sweep_rail(
        profile: BsplineCurve<Point3>,
        rail: &BsplineCurve<Point3>,
        n_sections: usize,
    ) -> BsplineSurface<Point3> {
        assert!(n_sections >= 2, "sweep_rail requires at least 2 sections");

        let (t_start, t_end) = rail.range_tuple();
        let rail_origin = rail.subs(t_start);
        let tangent0 = rail.derivative(t_start);
        let t0_len = tangent0.magnitude();

        let sections: Vec<BsplineCurve<Point3>> = (0..n_sections)
            .map(|i| {
                let t = t_start + (t_end - t_start) * i as f64 / (n_sections - 1) as f64;
                let rail_pt = rail.subs(t);
                let tangent_i = rail.derivative(t);
                let translation = rail_pt - rail_origin;

                // Compute rotation from initial tangent to current tangent.
                let rotation = if t0_len.so_small() || tangent_i.magnitude().so_small() {
                    Matrix3::from_value(1.0)
                } else {
                    rotation_between(tangent0, tangent_i)
                };

                let mut section = profile.clone();
                section.transform_control_points(|pt| {
                    let local = *pt - rail_origin;
                    let rotated = rotation * local;
                    *pt = rail_origin + rotated + translation;
                });
                section
            })
            .collect();

        BsplineSurface::skin(sections)
    }

    /// Creates a surface by sweeping a single profile along two rail curves (birail1).
    ///
    /// At each of `n_sections` uniformly sampled parameter values along the rails,
    /// the profile is affinely transformed so that its start aligns with `rail1`
    /// and its end aligns with `rail2`. The resulting sections are then skinned.
    ///
    /// The profile curve's start point should correspond to `rail1`'s start and
    /// its end point should correspond to `rail2`'s start. The two rails must
    /// share the same parameter domain.
    ///
    /// # Panics
    ///
    /// Panics if `n_sections < 2`.
    ///
    /// # Examples
    ///
    /// ```
    /// use monstertruck_geometry::prelude::*;
    ///
    /// // Two parallel straight rails separated along x.
    /// let rail1 = BsplineCurve::new(
    ///     KnotVector::bezier_knot(1),
    ///     vec![Point3::new(-1.0, 0.0, 0.0), Point3::new(-1.0, 0.0, 5.0)],
    /// );
    /// let rail2 = BsplineCurve::new(
    ///     KnotVector::bezier_knot(1),
    ///     vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 5.0)],
    /// );
    /// // Profile connects the rail starts.
    /// let profile = BsplineCurve::new(
    ///     KnotVector::bezier_knot(1),
    ///     vec![Point3::new(-1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
    /// );
    /// let surface = BsplineSurface::birail1(profile, &rail1, &rail2, 3);
    /// // At v=0 (rail start), corners match.
    /// assert_near2!(surface.subs(0.0, 0.0), Point3::new(-1.0, 0.0, 0.0));
    /// assert_near2!(surface.subs(1.0, 0.0), Point3::new(1.0, 0.0, 0.0));
    /// // At v=1 (rail end), corners match.
    /// assert_near2!(surface.subs(0.0, 1.0), Point3::new(-1.0, 0.0, 5.0));
    /// assert_near2!(surface.subs(1.0, 1.0), Point3::new(1.0, 0.0, 5.0));
    /// ```
    pub fn birail1(
        profile: BsplineCurve<Point3>,
        rail1: &BsplineCurve<Point3>,
        rail2: &BsplineCurve<Point3>,
        n_sections: usize,
    ) -> BsplineSurface<Point3> {
        assert!(n_sections >= 2, "birail1 requires at least 2 sections");

        let (r_start, r_end) = rail1.range_tuple();
        let (u_start, u_end) = profile.range_tuple();
        let p_start = profile.subs(u_start);
        let p_end = profile.subs(u_end);
        let chord = p_end - p_start;
        let chord_len = chord.magnitude();

        let sections: Vec<BsplineCurve<Point3>> = (0..n_sections)
            .map(|i| {
                let t = r_start + (r_end - r_start) * i as f64 / (n_sections - 1) as f64;
                let r1_pt = rail1.subs(t);
                let r2_pt = rail2.subs(t);
                let target_chord = r2_pt - r1_pt;
                let target_len = target_chord.magnitude();

                // Scale factor from profile chord to target chord.
                let scale = if chord_len.so_small() {
                    1.0
                } else {
                    target_len / chord_len
                };

                // Rotation from profile chord to target chord direction.
                let rotation = if chord_len.so_small() || target_len.so_small() {
                    Matrix3::from_value(1.0)
                } else {
                    rotation_between(chord, target_chord)
                };

                let mut section = profile.clone();
                section.transform_control_points(|pt| {
                    let local = *pt - p_start;
                    let transformed = rotation * local * scale;
                    *pt = r1_pt + transformed;
                });
                section
            })
            .collect();

        BsplineSurface::skin(sections)
    }

    /// Creates a surface by blending two profiles along two rail curves (birail2).
    ///
    /// At each of `n_sections` uniformly sampled parameter values along the rails,
    /// two section curves are computed: one from each profile, both affinely
    /// transformed to span from `rail1` to `rail2` at that parameter. The final
    /// section is a linear blend of the two transformed profiles, weighted by the
    /// normalized v-parameter.
    ///
    /// At v=0, the section matches `profile1`'s shape; at v=1, `profile2`'s shape.
    ///
    /// # Panics
    ///
    /// Panics if `n_sections < 2`.
    ///
    /// # Examples
    ///
    /// ```
    /// use monstertruck_geometry::prelude::*;
    ///
    /// // Two parallel straight rails.
    /// let rail1 = BsplineCurve::new(
    ///     KnotVector::bezier_knot(1),
    ///     vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 4.0)],
    /// );
    /// let rail2 = BsplineCurve::new(
    ///     KnotVector::bezier_knot(1),
    ///     vec![Point3::new(2.0, 0.0, 0.0), Point3::new(2.0, 0.0, 4.0)],
    /// );
    /// // Two identical straight profiles (result is a ruled surface).
    /// let profile1 = BsplineCurve::new(
    ///     KnotVector::bezier_knot(1),
    ///     vec![Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)],
    /// );
    /// let profile2 = BsplineCurve::new(
    ///     KnotVector::bezier_knot(1),
    ///     vec![Point3::new(0.0, 0.0, 4.0), Point3::new(2.0, 0.0, 4.0)],
    /// );
    /// let surface = BsplineSurface::birail2(profile1, profile2, &rail1, &rail2, 3);
    /// // Corners.
    /// assert_near2!(surface.subs(0.0, 0.0), Point3::new(0.0, 0.0, 0.0));
    /// assert_near2!(surface.subs(1.0, 0.0), Point3::new(2.0, 0.0, 0.0));
    /// assert_near2!(surface.subs(0.0, 1.0), Point3::new(0.0, 0.0, 4.0));
    /// assert_near2!(surface.subs(1.0, 1.0), Point3::new(2.0, 0.0, 4.0));
    /// ```
    pub fn birail2(
        profile1: BsplineCurve<Point3>,
        profile2: BsplineCurve<Point3>,
        rail1: &BsplineCurve<Point3>,
        rail2: &BsplineCurve<Point3>,
        n_sections: usize,
    ) -> BsplineSurface<Point3> {
        assert!(n_sections >= 2, "birail2 requires at least 2 sections");

        let (r_start, r_end) = rail1.range_tuple();

        let (u1_start, u1_end) = profile1.range_tuple();
        let p1_start = profile1.subs(u1_start);
        let p1_end = profile1.subs(u1_end);
        let chord1 = p1_end - p1_start;
        let chord1_len = chord1.magnitude();

        let (u2_start, u2_end) = profile2.range_tuple();
        let p2_start = profile2.subs(u2_start);
        let p2_end = profile2.subs(u2_end);
        let chord2 = p2_end - p2_start;
        let chord2_len = chord2.magnitude();

        // Make profiles compatible so we can blend control points.
        let mut compat_profiles = vec![profile1, profile2];
        compat::make_curves_compatible(&mut compat_profiles)
            .expect("birail2: profile compatibility normalization failed");

        let sections: Vec<BsplineCurve<Point3>> = (0..n_sections)
            .map(|i| {
                let v = i as f64 / (n_sections - 1) as f64;
                let t = r_start + (r_end - r_start) * v;
                let r1_pt = rail1.subs(t);
                let r2_pt = rail2.subs(t);
                let target_chord = r2_pt - r1_pt;
                let target_len = target_chord.magnitude();

                // Transform profile1 to span r1->r2.
                let transform_profile =
                    |prof: &BsplineCurve<Point3>, p_s: Point3, ch: Vector3, ch_len: f64| {
                        let scale = if ch_len.so_small() {
                            1.0
                        } else {
                            target_len / ch_len
                        };
                        let rotation = if ch_len.so_small() || target_len.so_small() {
                            Matrix3::from_value(1.0)
                        } else {
                            rotation_between(ch, target_chord)
                        };
                        let mut s = prof.clone();
                        s.transform_control_points(|pt| {
                            let local = *pt - p_s;
                            let transformed = rotation * local * scale;
                            *pt = r1_pt + transformed;
                        });
                        s
                    };

                let s1 = transform_profile(&compat_profiles[0], p1_start, chord1, chord1_len);
                let s2 = transform_profile(&compat_profiles[1], p2_start, chord2, chord2_len);

                // Blend: (1-v) * s1 + v * s2.
                let cp1 = s1.control_points();
                let cp2 = s2.control_points();
                let blended_cp: Vec<Point3> = cp1
                    .iter()
                    .zip(cp2.iter())
                    .map(|(a, b)| *a + (*b - *a) * v)
                    .collect();

                BsplineCurve::new_unchecked(s1.knot_vector().clone(), blended_cp)
            })
            .collect();

        BsplineSurface::skin(sections)
    }
}

/// Computes a rotation [`Matrix3`] that rotates vector `from` to align with `to`.
///
/// Uses Rodrigues' rotation formula. Returns the identity if the vectors
/// are nearly parallel or either has near-zero magnitude.
fn rotation_between(from: Vector3, to: Vector3) -> Matrix3 {
    let f = from.normalize();
    let t = to.normalize();
    let dot = f.dot(t);

    // Nearly parallel -- no rotation needed.
    if (dot - 1.0).abs() < TOLERANCE {
        return Matrix3::from_value(1.0);
    }

    // Nearly anti-parallel -- rotate 180 degrees around an arbitrary perpendicular axis.
    if (dot + 1.0).abs() < TOLERANCE {
        let perp = if f.x.abs() < 0.9 {
            Vector3::unit_x()
        } else {
            Vector3::unit_y()
        };
        let axis = f.cross(perp).normalize();
        return Matrix3::from_axis_angle(axis, Rad(std::f64::consts::PI));
    }

    let axis = f.cross(t).normalize();
    let angle = Rad(dot.acos());
    Matrix3::from_axis_angle(axis, angle)
}

impl<P: ControlPoint<f64> + Tolerance> BsplineSurface<P> {
    /// Creates a Gordon surface from two families of compatible curves.
    ///
    /// Given `n` u-direction curves and `m` v-direction curves that form a grid
    /// with known intersection points, the Gordon surface interpolates all
    /// input curves exactly using the boolean sum formula:
    ///
    /// `G(u,v) = skin_u(u,v) + skin_v(u,v) - tensor(u,v)`
    ///
    /// The `points` grid must have dimensions `[n][m]` where `points[i][j]`
    /// is the intersection of `u_curves[i]` and `v_curves[j]`.
    ///
    /// # Panics
    ///
    /// Panics if `u_curves` or `v_curves` is empty, or if `points` dimensions
    /// don't match `[u_curves.len()][v_curves.len()]`.
    ///
    /// # Examples
    ///
    /// ```
    /// use monstertruck_geometry::prelude::*;
    ///
    /// // Two u-curves and two v-curves forming a bilinear patch.
    /// let u0 = BsplineCurve::new(
    ///     KnotVector::bezier_knot(1),
    ///     vec![Vector2::new(0.0, 0.0), Vector2::new(1.0, 0.0)],
    /// );
    /// let u1 = BsplineCurve::new(
    ///     KnotVector::bezier_knot(1),
    ///     vec![Vector2::new(0.0, 1.0), Vector2::new(1.0, 1.0)],
    /// );
    /// let v0 = BsplineCurve::new(
    ///     KnotVector::bezier_knot(1),
    ///     vec![Vector2::new(0.0, 0.0), Vector2::new(0.0, 1.0)],
    /// );
    /// let v1 = BsplineCurve::new(
    ///     KnotVector::bezier_knot(1),
    ///     vec![Vector2::new(1.0, 0.0), Vector2::new(1.0, 1.0)],
    /// );
    /// let points = vec![
    ///     vec![Vector2::new(0.0, 0.0), Vector2::new(1.0, 0.0)],
    ///     vec![Vector2::new(0.0, 1.0), Vector2::new(1.0, 1.0)],
    /// ];
    /// let gordon = BsplineSurface::gordon(
    ///     vec![u0, u1],
    ///     vec![v0, v1],
    ///     &points,
    /// );
    /// assert_near2!(gordon.subs(0.0, 0.0), Vector2::new(0.0, 0.0));
    /// assert_near2!(gordon.subs(1.0, 0.0), Vector2::new(1.0, 0.0));
    /// assert_near2!(gordon.subs(0.0, 1.0), Vector2::new(0.0, 1.0));
    /// assert_near2!(gordon.subs(1.0, 1.0), Vector2::new(1.0, 1.0));
    /// ```
    pub fn gordon(
        u_curves: Vec<BsplineCurve<P>>,
        v_curves: Vec<BsplineCurve<P>>,
        points: &[Vec<P>],
    ) -> BsplineSurface<P> {
        let n = u_curves.len();
        let m = v_curves.len();
        assert!(!u_curves.is_empty(), "gordon requires at least one u-curve");
        assert!(!v_curves.is_empty(), "gordon requires at least one v-curve");
        assert_eq!(points.len(), n, "points rows must match u_curves count");
        assert!(
            points.iter().all(|row| row.len() == m),
            "each points row must have v_curves.len() columns",
        );

        // S_u: skin the u-curves (v parameterizes across sections).
        let s_u = BsplineSurface::skin(u_curves);

        // S_v: skin the v-curves (u parameterizes across sections),
        // then swap axes so u/v orientation matches S_u.
        let mut s_v = BsplineSurface::skin(v_curves);
        s_v.swap_axes();

        // T: tensor product surface interpolating the grid points.
        // Build as degree-1 in both u and v, using the grid points directly.
        let n_u = n;
        let n_v = m;
        let mut u_knots = Vec::with_capacity(n_u + 2);
        u_knots.push(0.0);
        (0..n_u).for_each(|i| u_knots.push(i as f64 / (n_u - 1).max(1) as f64));
        u_knots.push(1.0);

        let mut v_knots = Vec::with_capacity(n_v + 2);
        v_knots.push(0.0);
        (0..n_v).for_each(|j| v_knots.push(j as f64 / (n_v - 1).max(1) as f64));
        v_knots.push(1.0);

        let knot_u = KnotVector::from(u_knots);
        let knot_v = KnotVector::from(v_knots);
        // Control points: rows indexed by u-column (matching skin layout).
        let t_cp: Vec<Vec<P>> = (0..n_v)
            .map(|j| (0..n_u).map(|i| points[i][j]).collect())
            .collect();
        let tensor = BsplineSurface::new_unchecked((knot_u, knot_v), t_cp);

        // Make all three surfaces compatible.
        let mut surfaces = vec![s_u, s_v, tensor];
        compat::make_surfaces_compatible(&mut surfaces)
            .expect("gordon: surface compatibility normalization failed");

        // G = S_u + S_v - T (boolean sum).
        let cp_u = surfaces[0].control_points();
        let cp_v = surfaces[1].control_points();
        let cp_t = surfaces[2].control_points();
        let rows = cp_u.len();
        let cols = cp_u[0].len();
        let result_cp: Vec<Vec<P>> = (0..rows)
            .map(|i| {
                (0..cols)
                    .map(|j| cp_u[i][j] + (cp_v[i][j] - cp_t[i][j]))
                    .collect()
            })
            .collect();

        BsplineSurface::new_unchecked(surfaces[0].knot_vectors().clone(), result_cp)
    }

    /// Creates a surface by its boundary.
    /// # Examples
    /// ```
    /// use monstertruck_geometry::prelude::*;
    /// let curve0 = BsplineCurve::new(
    ///     KnotVector::bezier_knot(1),
    ///     vec![Vector2::new(0.0, 0.0), Vector2::new(1.0, 0.0)],
    /// );
    /// let curve1 = BsplineCurve::new(
    ///     KnotVector::bezier_knot(2),
    ///     vec![Vector2::new(1.0, 0.0), Vector2::new(2.0, 0.5), Vector2::new(1.0, 1.0)],
    /// );
    /// let curve2 = BsplineCurve::new(
    ///     KnotVector::bezier_knot(1),
    ///     vec![Vector2::new(1.0, 1.0), Vector2::new(0.0, 1.0)],
    /// );
    /// let curve3 = BsplineCurve::new(
    ///     KnotVector::bezier_knot(2),
    ///     vec![Vector2::new(0.0, 1.0), Vector2::new(-1.0, 0.5), Vector2::new(0.0, 0.0)],
    /// );
    /// let surface = BsplineSurface::by_boundary(curve0, curve1, curve2, curve3);
    /// assert_eq!(
    ///     surface.control_points(),
    ///     &vec![
    ///         vec![Vector2::new(0.0, 0.0), Vector2::new(-1.0, 0.5), Vector2::new(0.0, 1.0)],
    ///         vec![Vector2::new(1.0, 0.0), Vector2::new(2.0, 0.5), Vector2::new(1.0, 1.0)],
    ///     ],
    /// );
    /// ```
    /// # Remarks
    /// If the end points of curves are not connected, `curve1` and `curve3` take precedence. i.e.
    /// `curve1` and `curve3` are contained in the boundary of the surface and `curve0` and
    /// `curve2` are not contained.
    /// ```
    /// use monstertruck_geometry::prelude::*;
    /// let curve0 = BsplineCurve::new(
    ///     KnotVector::bezier_knot(1),
    ///     vec![Vector2::new(0.0, 0.0), Vector2::new(1.0, 0.0)],
    /// );
    /// let curve1 = BsplineCurve::new(
    ///     KnotVector::bezier_knot(2),
    ///     vec![Vector2::new(2.0, 0.0), Vector2::new(3.0, 0.5), Vector2::new(2.0, 1.0)],
    /// );
    /// let curve2 = BsplineCurve::new(
    ///     KnotVector::bezier_knot(1),
    ///     vec![Vector2::new(1.0, 1.0), Vector2::new(0.0, 1.0)],
    /// );
    /// let curve3 = BsplineCurve::new(
    ///     KnotVector::bezier_knot(2),
    ///     vec![Vector2::new(-1.0, 1.0), Vector2::new(-2.0, 0.5), Vector2::new(-1.0, 0.0)],
    /// );
    /// let surface = BsplineSurface::by_boundary(
    ///     curve0.clone(),
    ///     curve1.clone(),
    ///     curve2.clone(),
    ///     curve3.clone()
    /// );
    /// assert_ne!(surface.subs(0.0, 0.0), curve0.subs(0.0));
    /// assert_eq!(surface.subs(0.0, 0.0), curve3.subs(1.0));
    /// ```
    pub fn by_boundary(
        mut curve0: BsplineCurve<P>,
        mut curve1: BsplineCurve<P>,
        mut curve2: BsplineCurve<P>,
        mut curve3: BsplineCurve<P>,
    ) -> BsplineSurface<P> {
        curve2.invert();
        curve3.invert();
        curve0.syncro_degree(&mut curve2);
        curve0.optimize();
        curve2.optimize();
        curve0.syncro_knots(&mut curve2);
        curve1.syncro_degree(&mut curve3);
        curve1.optimize();
        curve3.optimize();
        curve1.syncro_knots(&mut curve3);

        let knot_vecs = (curve0.knot_vector().clone(), curve3.knot_vector().clone());
        let mut control_points = vec![curve3.control_points().clone()];
        let n = curve0.control_points().len();
        let m = curve3.control_points().len();
        for i in 1..(n - 1) {
            let u = (i as f64) / (n as f64);
            let pt0 = curve2.control_points[i]
                + (curve0.control_points[i] - curve2.control_points[i]) * u;
            let mut new_row = vec![*curve0.control_point(i)];
            for j in 1..(m - 1) {
                let v = (j as f64) / (m as f64);
                let pt1 = curve1.control_points[j]
                    + (curve3.control_points[j] - curve1.control_points[j]) * v;
                new_row.push(pt0 + (pt1 - pt0) / 2.0);
            }
            new_row.push(*curve2.control_point(i));
            control_points.push(new_row);
        }
        control_points.push(curve1.control_points().clone());
        BsplineSurface::new(knot_vecs, control_points)
    }
}
