//! A mesh whose vertex row lies on another mesh's plane, a few ulps either side of it,
//! still meets that plane along the whole row.

use monstertruck_meshing::prelude::*;

fn covered_length(segments: &[(Point3, Point3)]) -> f64 {
    let mut spans: Vec<(f64, f64)> = segments
        .iter()
        .map(|(a, b)| (a.x.min(b.x), a.x.max(b.x)))
        .collect();
    spans.sort_by(|a, b| a.0.total_cmp(&b.0));
    spans
        .into_iter()
        .fold(Vec::<(f64, f64)>::new(), |mut merged, (low, high)| {
            match merged.last_mut() {
                Some(last) if low <= last.1 + 1.0e-9 => last.1 = last.1.max(high),
                _ => merged.push((low, high)),
            }
            merged
        })
        .iter()
        .map(|(low, high)| high - low)
        .sum()
}

#[test]
fn a_row_on_the_plane_meets_it_everywhere() {
    let plane = PolygonMesh::new(
        StandardAttributes {
            positions: vec![
                Point3::new(-10.0, -10.0, 0.0),
                Point3::new(10.0, -10.0, 0.0),
                Point3::new(10.0, 10.0, 0.0),
                Point3::new(-10.0, 10.0, 0.0),
            ],
            ..Default::default()
        },
        Faces::from_iter([[0, 1, 2], [0, 2, 3]]),
    );
    let columns = 11;
    let off = |i: usize| if i < columns / 2 { -2.0e-16 } else { 2.0e-16 };
    let positions: Vec<Point3> = [-1.0, 0.0, 1.0]
        .into_iter()
        .flat_map(|z: f64| {
            (0..columns).map(move |i| {
                let x = -5.0 + i as f64;
                Point3::new(x, 0.0, if z == 0.0 { off(i) } else { z })
            })
        })
        .collect();
    let at = |row: usize, column: usize| row * columns + column;
    let faces = Faces::from_iter((0..2).flat_map(|row| {
        (0..columns - 1).flat_map(move |i| {
            [
                [at(row, i), at(row, i + 1), at(row + 1, i + 1)],
                [at(row, i), at(row + 1, i + 1), at(row + 1, i)],
            ]
        })
    }));
    let strip = PolygonMesh::new(
        StandardAttributes {
            positions,
            ..Default::default()
        },
        faces,
    );
    let segments = strip.extract_interference(&plane);
    let covered = covered_length(&segments);
    assert!((covered - 10.0).abs() < 1.0e-9, "covered {covered} of 10");
}
