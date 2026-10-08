//! Edges a fillet chain makes at its corners lie on the faces they bound.

use monstertruck_modeling::*;

fn worst_gap(shell: &Shell) -> f64 {
    shell
        .face_iter()
        .flat_map(|face| {
            let surface = face.oriented_surface();
            face.edge_iter()
                .flat_map(|edge| {
                    let curve = edge.curve();
                    let (t0, t1) = curve.range_tuple();
                    (0..=24)
                        .map(|i| curve.subs(t0 + (t1 - t0) * i as f64 / 24.0))
                        .collect::<Vec<_>>()
                })
                .map(|point| {
                    surface
                        .search_nearest_parameter(point, None, 100)
                        .map_or(f64::INFINITY, |(u, v)| surface.subs(u, v).distance(point))
                })
                .collect::<Vec<_>>()
        })
        .fold(0.0, f64::max)
}

fn filleted_rim(radius: f64, profile: FilletProfile) -> Shell {
    let plate = primitive::cuboid(BoundingBox::from_iter([
        Point3::new(-20.0, -15.0, 0.0),
        Point3::new(20.0, 15.0, 3.0),
    ]));
    let mut shell = plate.boundaries()[0].clone();
    let rim = shell
        .edge_iter()
        .filter(|edge| edge.front().point().z > 2.9 && edge.back().point().z > 2.9)
        .fold(Vec::<Edge>::new(), |mut edges, edge| {
            if !edges.iter().any(|known| known.id() == edge.id()) {
                edges.push(edge);
            }
            edges
        });
    assert_eq!(rim.len(), 4);
    let options = FilletOptions::constant(radius).with_profile(profile);
    fillet_edges(&mut shell, &rim, Some(&options)).expect("fillet the rim");
    shell
}

#[test]
fn mitred_round_corners_lie_on_both_fillets() {
    let gap = worst_gap(&filleted_rim(1.0, FilletProfile::Round));
    assert!(gap < 1.0e-6, "a corner edge strays {gap} from its faces");
}
