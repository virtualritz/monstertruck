use itertools::Itertools;
use monstertruck_geometry::prelude::*;
use monstertruck_io::step::save::{DisplayByStep, StepCurve, StepLength, StepModels};
use monstertruck_meshing::prelude::*;

use super::geometry::*;
use super::types::*;

use monstertruck_traits::ParametricSurface;
use std::fmt;
use std::fs;
use std::path::{Path, PathBuf};

use super::{
    FilletError, FilletOptions, FilletProfile, RadiusSpec, fillet, fillet_along_wire, fillet_edges,
    fillet_edges_generic, fillet_with_side, topology,
};

mod accuracy;
mod edges;
mod profile;
mod radius;
mod surface;

// CSG/boolean-result fillet coverage lives in
// monstertruck-modeling/tests/fillet_test.rs (feature `fillet`): the
// modeling-curve conversions are feature-gated there, and enabling them in
// this crate's unit tests would pull in a second monstertruck-fillet whose
// fillet types cannot unify with the crate under test.

fn test_artifact_dir() -> PathBuf {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let workspace_dir = manifest_dir.parent().unwrap_or_else(|| Path::new("."));
    let dir = workspace_dir.join("target").join("tests").join("fillet");
    fs::create_dir_all(&dir).unwrap();
    dir
}

fn step_polyline(curve: &Curve) -> PolylineCurve<Point3> {
    let (t0, t1) = curve.range_tuple();
    let points = (0..=32)
        .map(|i| t0 + (t1 - t0) * (i as f64) / 32.0)
        .map(|t| curve.subs(t))
        .collect();
    PolylineCurve(points)
}

impl DisplayByStep for Curve {
    fn fmt(&self, idx: usize, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        DisplayByStep::fmt(&step_polyline(self), idx, f)
    }
}

impl StepLength for Curve {
    fn step_length(&self) -> usize { step_polyline(self).step_length() }
}

impl StepCurve for Curve {}

fn format_entity(entity: &(impl DisplayByStep + StepLength), idx: usize) -> (String, usize) {
    struct EntityFmt<'a, T>(&'a T, usize);

    impl<T: DisplayByStep> fmt::Display for EntityFmt<'_, T> {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            DisplayByStep::fmt(self.0, self.1, f)
        }
    }

    let body = EntityFmt(entity, idx).to_string();
    (body, idx + entity.step_length())
}

fn format_step_file(name: &str, data_section: &str) -> String {
    format!(
        "ISO-10303-21;\n\
         HEADER;\n\
         FILE_DESCRIPTION(('Fillet Debug: {name}'), '2;1');\n\
         FILE_NAME('{name}.step', '', (''), (''), '', 'monstertruck', '');\n\
         FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\n\
         ENDSEC;\n\
         DATA;\n\
         {data_section}\
         ENDSEC;\n\
         END-ISO-10303-21;\n"
    )
}

fn write_step(path: PathBuf, name: &str, data_section: String) {
    fs::write(path, format_step_file(name, &data_section)).unwrap();
}

fn dump_shell_step(name: &str, shell: &Shell) {
    let compressed = shell.compress();
    let mut models = StepModels::default();
    models.push_shell(&compressed);
    write_step(
        test_artifact_dir().join(format!("{name}.step")),
        name,
        models.to_string(),
    );
}

fn dump_surface_step<S>(name: &str, surface: &S)
where S: DisplayByStep + StepLength {
    let (entity, _) = format_entity(surface, 1);
    write_step(
        test_artifact_dir().join(format!("{name}.step")),
        name,
        entity,
    );
}

fn dump_surface_pair_step<S>(name: &str, surface0: &S, surface1: &S)
where S: DisplayByStep + StepLength {
    let (entity0, next) = format_entity(surface0, 1);
    let (entity1, _) = format_entity(surface1, next);
    write_step(
        test_artifact_dir().join(format!("{name}.step")),
        name,
        format!("{entity0}{entity1}"),
    );
}

/// Helper: builds a box-like shell with `plane()` and `line()` helpers.
/// Returns `(shell, edges, vertices)`.
fn build_box_shell() -> (Shell, [Edge; 12], Vec<Vertex>) {
    let p = [
        Point3::new(0.0, 0.0, 1.0),
        Point3::new(1.0, 0.0, 1.0),
        Point3::new(1.0, 1.0, 1.0),
        Point3::new(0.0, 1.0, 1.0),
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    ];
    let v = Vertex::from_points(p);

    let line = |i: usize, j: usize| {
        let bsp = BsplineCurve::new(KnotVector::bezier_knot(1), vec![p[i], p[j]]);
        Edge::new(&v[i], &v[j], NurbsCurve::from(bsp).into())
    };
    let edge = [
        line(0, 1), // 0
        line(1, 2), // 1
        line(2, 3), // 2
        line(3, 0), // 3
        line(0, 4), // 4
        line(1, 5), // 5
        line(2, 6), // 6
        line(3, 7), // 7
        line(4, 5), // 8
        line(5, 6), // 9
        line(6, 7), // 10
        line(7, 4), // 11
    ];

    let plane = |i: usize, j: usize, k: usize, l: usize| {
        let control_points = vec![vec![p[i], p[l]], vec![p[j], p[k]]];
        let knot_vec = KnotVector::bezier_knot(1);
        let knot_vecs = (knot_vec.clone(), knot_vec);
        let bsp = BsplineSurface::new(knot_vecs, control_points);

        let wire: Wire = [i, j, k, l]
            .into_iter()
            .circular_tuple_windows()
            .map(|(i, j)| {
                edge.iter()
                    .find_map(|edge| {
                        if edge.front() == &v[i] && edge.back() == &v[j] {
                            Some(edge.clone())
                        } else if edge.back() == &v[i] && edge.front() == &v[j] {
                            Some(edge.inverse())
                        } else {
                            None
                        }
                    })
                    .unwrap()
            })
            .collect();
        Face::new(vec![wire], bsp.into())
    };

    // Top, front, right, back (partial box -- 4 faces sharing edges).
    let shell: Shell = [
        plane(0, 1, 2, 3), // face 0: top
        plane(1, 0, 4, 5), // face 1: front
        plane(2, 1, 5, 6), // face 2: right
        plane(3, 2, 6, 7), // face 3: back
    ]
    .into();

    (shell, edge, v)
}

/// Helper: builds a 5-face cuboid (top + 4 sides, no bottom) with 12 edges.
fn build_5face_box() -> (Shell, [Edge; 12], Vec<Vertex>) {
    let p = [
        Point3::new(0.0, 0.0, 1.0),
        Point3::new(1.0, 0.0, 1.0),
        Point3::new(1.0, 1.0, 1.0),
        Point3::new(0.0, 1.0, 1.0),
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    ];
    let v = Vertex::from_points(p);

    let line = |i: usize, j: usize| {
        let bsp = BsplineCurve::new(KnotVector::bezier_knot(1), vec![p[i], p[j]]);
        Edge::new(&v[i], &v[j], NurbsCurve::from(bsp).into())
    };
    let edge = [
        line(0, 1), // 0: top front
        line(1, 2), // 1: top right
        line(2, 3), // 2: top back
        line(3, 0), // 3: top left
        line(0, 4), // 4
        line(1, 5), // 5
        line(2, 6), // 6
        line(3, 7), // 7
        line(4, 5), // 8: bottom front
        line(5, 6), // 9: bottom right
        line(6, 7), // 10: bottom back
        line(7, 4), // 11: bottom left
    ];

    let plane = |i: usize, j: usize, k: usize, l: usize| {
        let control_points = vec![vec![p[i], p[l]], vec![p[j], p[k]]];
        let knot_vec = KnotVector::bezier_knot(1);
        let bsp = BsplineSurface::new((knot_vec.clone(), knot_vec), control_points);
        let wire: Wire = [i, j, k, l]
            .into_iter()
            .circular_tuple_windows()
            .map(|(a, b)| {
                edge.iter()
                    .find_map(|e| {
                        if e.front() == &v[a] && e.back() == &v[b] {
                            Some(e.clone())
                        } else if e.back() == &v[a] && e.front() == &v[b] {
                            Some(e.inverse())
                        } else {
                            None
                        }
                    })
                    .unwrap()
            })
            .collect();
        Face::new(vec![wire], bsp.into())
    };

    let shell: Shell = [
        plane(0, 1, 2, 3), // face 0: top
        plane(1, 0, 4, 5), // face 1: front
        plane(2, 1, 5, 6), // face 2: right
        plane(3, 2, 6, 7), // face 3: back
        plane(0, 3, 7, 4), // face 4: left
    ]
    .into();

    (shell, edge, v)
}

/// Helper: builds a 6-face closed cuboid with 12 edges.
fn build_6face_box() -> (Shell, [Edge; 12], Vec<Vertex>) {
    let p = [
        Point3::new(0.0, 0.0, 1.0),
        Point3::new(1.0, 0.0, 1.0),
        Point3::new(1.0, 1.0, 1.0),
        Point3::new(0.0, 1.0, 1.0),
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    ];
    let v = Vertex::from_points(p);

    let line = |i: usize, j: usize| {
        let bsp = BsplineCurve::new(KnotVector::bezier_knot(1), vec![p[i], p[j]]);
        Edge::new(&v[i], &v[j], NurbsCurve::from(bsp).into())
    };
    let edge = [
        line(0, 1), // 0: top front
        line(1, 2), // 1: top right
        line(2, 3), // 2: top back
        line(3, 0), // 3: top left
        line(0, 4), // 4
        line(1, 5), // 5
        line(2, 6), // 6
        line(3, 7), // 7
        line(4, 5), // 8: bottom front
        line(5, 6), // 9: bottom right
        line(6, 7), // 10: bottom back
        line(7, 4), // 11: bottom left
    ];

    let plane = |i: usize, j: usize, k: usize, l: usize| {
        let control_points = vec![vec![p[i], p[l]], vec![p[j], p[k]]];
        let knot_vec = KnotVector::bezier_knot(1);
        let bsp = BsplineSurface::new((knot_vec.clone(), knot_vec), control_points);
        let wire: Wire = [i, j, k, l]
            .into_iter()
            .circular_tuple_windows()
            .map(|(a, b)| {
                edge.iter()
                    .find_map(|e| {
                        if e.front() == &v[a] && e.back() == &v[b] {
                            Some(e.clone())
                        } else if e.back() == &v[a] && e.front() == &v[b] {
                            Some(e.inverse())
                        } else {
                            None
                        }
                    })
                    .unwrap()
            })
            .collect();
        Face::new(vec![wire], bsp.into())
    };

    let shell: Shell = [
        plane(0, 1, 2, 3), // face 0: top
        plane(1, 0, 4, 5), // face 1: front
        plane(2, 1, 5, 6), // face 2: right
        plane(3, 2, 6, 7), // face 3: back
        plane(0, 3, 7, 4), // face 4: left
        plane(5, 4, 7, 6), // face 5: bottom
    ]
    .into();

    (shell, edge, v)
}
