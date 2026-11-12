//! Our STEP reader against the cadmpeg bridge, on the NIST MBE PMI parts.
//!
//! `cadmpeg_versus_ours_on_the_real_corpus.rs` compares what each reader's
//! DECODER recovers. This row goes one step further and compares what each
//! reader hands the KERNEL: faces that survive conversion to monstertruck's
//! B-rep, and the volume those faces enclose once tessellated. The two paths
//! share no decoder and no converter, so agreement means something, and a
//! third column -- the volume the file DECLARES in its own validation
//! properties, read by cadmpeg -- is an oracle that needs neither reader.
//!
//! # The population
//!
//! NIST's CTC and FTC test parts (US government work, public domain under 17
//! U.S.C. 105). They are not vendored here; point `MT_NIST_STEP_DIR` at a
//! directory holding the `.stp` files, for example the `corpus/nist-pmi/step`
//! directory of a cadmpeg checkout:
//!
//! ```text
//! MT_NIST_STEP_DIR=/path/to/cadmpeg/corpus/nist-pmi/step \
//!     cargo nextest run -p monstertruck-io --features iges --run-ignored all \
//!     --test cadmpeg_versus_ours_on_nist --no-capture
//! ```
//!
//! # Units
//!
//! cadmpeg normalises every length to millimetres on decode; our reader keeps
//! the file's own unit. On the inch-unit parts (FTC-07, FTC-09) our volume is
//! therefore in cubic inches, and the "cadmpeg vs ours" ratio shows the factor
//! 25.4^3 = 16387.064 rather than a disagreement. The evidence note divides it
//! out.
//!
//! # This row REPORTS
//!
//! Like its sibling it asserts only internal consistency: every file listed is
//! present and produces exactly one row per reader. Which reader is closer to
//! the declared volume is the finding, and asserting it would freeze today's
//! answer. The measured table lives in the evidence note
//! `specs/028-iges-read/evidence/cadmpeg-0.6-port.md`.

#![cfg(feature = "cadmpeg")]

use monstertruck_io::cadmpeg::{ImportedBody, ImportedSheet};
use monstertruck_io::step::load::Table;
use monstertruck_meshing::prelude::*;
use monstertruck_topology::shell::ShellCondition;
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::path::PathBuf;

const FILES: [&str; 9] = [
    "nist_ctc_01_asme1_ap242-e1.stp",
    "nist_ctc_02_asme1_ap242-e2.stp",
    "nist_ctc_03_asme1_ap242-e2.stp",
    "nist_ctc_04_asme1_ap242-e1.stp",
    "nist_ctc_05_asme1_ap242-e1.stp",
    "nist_ftc_06_asme1_ap242-e2.stp",
    "nist_ftc_07_asme1_ap242-e2.stp",
    "nist_ftc_09_asme1_rd.stp",
    "nist_ftc_10_asme1_ap242-e2.stp",
];

/// Relative tessellation tolerance: a fraction of the part's bounding-box
/// diagonal, so a 10 mm part and a 1 m part are meshed alike.
const RELATIVE_TOLERANCE: f64 = 2.0e-4;

fn catching<T>(f: impl FnOnce() -> T) -> Result<T, String> {
    catch_unwind(AssertUnwindSafe(f)).map_err(|payload| {
        payload
            .downcast_ref::<&str>()
            .map(|s| (*s).to_string())
            .or_else(|| payload.downcast_ref::<String>().cloned())
            .unwrap_or_else(|| "<non-string panic>".to_string())
    })
}

/// The signed volume of a triangle mesh, about the mesh's own bounding-box
/// centre.
///
/// The divergence-theorem sum is origin-referenced and its terms nearly cancel
/// for a part far from the origin, so the positions are re-centred first. That
/// makes the result the mesh's volume to rounding, leaving the TESSELLATION as
/// the only approximation in the number.
fn centred_volume(mesh: &PolygonMesh) -> f64 {
    let positions = mesh.positions();
    if positions.is_empty() {
        return 0.0;
    }
    let mut low = positions[0];
    let mut high = positions[0];
    for p in positions {
        for axis in 0..3 {
            low[axis] = low[axis].min(p[axis]);
            high[axis] = high[axis].max(p[axis]);
        }
    }
    let centre = low.midpoint(high).to_vec();
    mesh.faces()
        .triangle_iter()
        .map(|[a, b, c]| {
            let (p, q, r) = (
                positions[a.pos] - centre,
                positions[b.pos] - centre,
                positions[c.pos] - centre,
            );
            p.to_vec().dot(q.to_vec().cross(r.to_vec()))
        })
        .sum::<f64>()
        / 6.0
}

/// What tessellating a set of shells gave.
#[derive(Default)]
struct Meshed {
    faces: usize,
    volume: f64,
    closed: usize,
    open: usize,
    panicked: usize,
}

impl Meshed {
    fn describe(&self) -> String {
        format!(
            "{} faces, V = {:.7e} ({} closed, {} open{})",
            self.faces,
            self.volume,
            self.closed,
            self.open,
            match self.panicked {
                0 => String::new(),
                count => format!(", {count} panicked"),
            }
        )
    }
}

/// Mesh one shell, closing it up the way the corpus tests do, and fold it in.
fn mesh_into(
    total: &mut Meshed,
    faces: usize,
    diagonal: f64,
    triangulate: impl FnOnce(f64) -> PolygonMesh,
) {
    total.faces += faces;
    match catching(|| triangulate(diagonal * RELATIVE_TOLERANCE)) {
        Err(_) => total.panicked += 1,
        Ok(mut mesh) => {
            mesh.put_together_same_attrs(TOLERANCE * 50.0)
                .remove_degenerate_faces();
            match mesh.shell_condition() {
                ShellCondition::Closed => total.closed += 1,
                _ => total.open += 1,
            }
            total.volume += centred_volume(&mesh);
        }
    }
}

fn diagonal_of(points: impl Iterator<Item = Point3>) -> f64 {
    let mut low = Point3::new(f64::MAX, f64::MAX, f64::MAX);
    let mut high = Point3::new(f64::MIN, f64::MIN, f64::MIN);
    for p in points {
        for axis in 0..3 {
            low[axis] = low[axis].min(p[axis]);
            high[axis] = high[axis].max(p[axis]);
        }
    }
    (high - low).magnitude().max(1.0)
}

/// Our reader: every shell record, converted to a trimmed shell and meshed.
/// Shells, not products: these parts are single bodies, so the per-file sum is
/// the part.
fn ours(path: &std::path::Path) -> Result<Meshed, String> {
    let bytes = std::fs::read(path).map_err(|e| e.to_string())?;
    let table = catching(|| Table::from_step_bytes(&bytes))?.map_err(|e| e.to_string())?;
    let mut total = Meshed::default();
    for step_shell in table.shell.values() {
        let shell = match catching(|| table.to_compressed_trimmed_shell(step_shell)) {
            Err(_) => {
                total.panicked += 1;
                continue;
            }
            Ok(Err(_)) => {
                total.open += 1;
                continue;
            }
            Ok(Ok(shell)) => shell,
        };
        let diagonal = diagonal_of(shell.vertices.iter().copied());
        let faces = shell.faces.len();
        mesh_into(&mut total, faces, diagonal, |tolerance| {
            shell.robust_triangulation(tolerance).to_polygon()
        });
    }
    Ok(total)
}

/// The bridge: decode with cadmpeg, convert, mesh every converted shell.
fn theirs(path: &std::path::Path) -> Result<(Meshed, String, Option<f64>), String> {
    let (ir, _) = catching(|| {
        let mut file = std::fs::File::open(path).map_err(|e| e.to_string())?;
        monstertruck_io::cadmpeg::step::decode_to_ir_reported(&mut file).map_err(|e| e.to_string())
    })??;
    let declared = ir
        .model
        .declared_measures
        .iter()
        .find_map(|measure| match measure.quantity {
            cadmpeg_ir::declared_measures::DeclaredQuantity::Volume { value } => Some(value.get()),
            _ => None,
        });
    let recovered = catching(|| monstertruck_io::cadmpeg::step::import_from_path(path))?
        .map_err(|e| e.to_string())?;
    let mut summary = recovered.summary();
    let mut reasons = std::collections::BTreeMap::<String, usize>::new();
    for refusal in &recovered.refused {
        *reasons.entry(refusal.reason.to_string()).or_default() += 1;
    }
    for (reason, count) in reasons {
        summary.push_str(&format!("\n           refused {count} x {reason}"));
    }
    let mut total = Meshed::default();
    let mut shells: Vec<&ImportedSheet> = Vec::new();
    for body in &recovered.bodies {
        match body {
            ImportedBody::Solid(solid) => shells.extend(solid.boundaries.iter()),
            ImportedBody::Sheet(sheet) => shells.push(sheet),
        }
    }
    for shell in shells {
        let diagonal = diagonal_of(shell.vertices.iter().copied());
        mesh_into(&mut total, shell.faces.len(), diagonal, |tolerance| {
            shell.robust_triangulation(tolerance).to_polygon()
        });
    }
    Ok((total, summary, declared))
}

fn relative(value: f64, reference: Option<f64>) -> String {
    match reference {
        Some(reference) if reference != 0.0 => {
            format!("{:+.4}%", 100.0 * (value - reference) / reference)
        }
        _ => "n/a".to_string(),
    }
}

#[test]
#[ignore = "corpus-gated: needs MT_NIST_STEP_DIR pointing at the NIST MBE PMI STEP files"]
fn our_reader_against_the_bridge_on_the_nist_parts() {
    let directory = PathBuf::from(std::env::var("MT_NIST_STEP_DIR").expect(
        "set MT_NIST_STEP_DIR to the directory holding the NIST .stp files -- an unset \
         variable must not read as an empty corpus",
    ));
    let mut rows = 0;
    for name in FILES {
        let path = directory.join(name);
        assert!(path.is_file(), "{} is missing", path.display());
        let ours = ours(&path);
        let theirs = theirs(&path);
        let declared = theirs.as_ref().ok().and_then(|(_, _, declared)| *declared);
        println!("{name}");
        println!(
            "  declared volume: {}",
            declared.map_or("none".to_string(), |v| format!("{v:.7e}"))
        );
        match &ours {
            Ok(meshed) => println!(
                "  ours:    {} | vs declared {}",
                meshed.describe(),
                relative(meshed.volume, declared)
            ),
            Err(error) => println!("  ours:    FAILED: {error}"),
        }
        match &theirs {
            Ok((meshed, summary, _)) => println!(
                "  cadmpeg: {} | vs declared {} | {summary}",
                meshed.describe(),
                relative(meshed.volume, declared)
            ),
            Err(error) => println!("  cadmpeg: FAILED: {error}"),
        }
        if let (Ok(a), Ok((b, _, _))) = (&ours, &theirs) {
            println!(
                "  faces ours/cadmpeg: {}/{} | volume cadmpeg vs ours {}",
                a.faces,
                b.faces,
                relative(b.volume, Some(a.volume))
            );
        }
        rows += 1;
    }
    assert_eq!(rows, FILES.len(), "a file was dropped from the table");
}
