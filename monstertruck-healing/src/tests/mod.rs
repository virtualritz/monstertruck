use super::*;

mod closed_edges;
mod closed_faces_cylinders;
mod closed_faces_holes;
mod non_simple_wires;
mod shell_orientation;

fn sp<S>(surface: &S, p: Point3, hint: Option<(f64, f64)>) -> Option<(f64, f64)>
where S: SearchParameter<SurfaceParameter, Point = Point3> {
    surface.search_parameter(p, hint, 10)
}

#[test]
#[cfg(feature = "step-test")]
fn step_import() {
    use monstertruck_io::step::load::*;
    const STEP_DIRECTORY: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../resources/step/");
    const STEP_FILES: &[&str] = &[
        "occt-cylinder.step",
        "occt-cone.step",
        "abc-0006.step",
        "abc-0008.step",
        "abc-0035.step",
    ];

    STEP_FILES.iter().for_each(|file_name| {
        println!("{file_name}");
        let path = [STEP_DIRECTORY, file_name].concat();
        let step_string = std::fs::read_to_string(path).unwrap();
        let table = Table::from_step(&step_string).unwrap();
        table.shell.values().for_each(|step_shell| {
            let mut cshell = table.to_compressed_shell(step_shell).unwrap();
            cshell.robust_split_closed_edges_and_faces(0.05);
            monstertruck_topology::Shell::extract(cshell).unwrap();
        });
    });
}
