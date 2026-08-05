//! Spec 012 U2 measurement harness -- how often does `normalize_uv` MOVE a
//! projected trim sample, and against what kind of range?
//!
//! Test-only. Drives `Curve3D::parameter_boundary_2d` directly over every
//! (face surface, boundary edge) pair of a loaded solid, which is a strict
//! SUPERSET of the pairs `monstertruck-solid`'s healing reaches
//! (`reattach_preserved_face_trims` only gets there after `exact` and
//! `regenerate_linear` both refuse). A zero measured here is therefore the
//! stronger claim; a non-zero is an upper bound on the live rate.
//!
//! Run:
//!
//! ```text
//! MT_STEP_DEBUG_UV_CLAMP=1 cargo nextest run -p monstertruck-step \
//!     --run-ignored all -E 'test(uv_clamp)' --no-capture --test-threads=1
//! ```
//!
//! The corpus probes are `#[ignore]`d and skip when the corpus is absent.

use super::{Curve3D, Surface, uv_clamp};
use crate::step::load::Table;
use monstertruck_geometry::prelude::{ParameterBoundary2D, Point2};
use monstertruck_topology::compress::CompressedShell;
use std::path::{Path, PathBuf};

/// The tolerance the real healing callers pass (`real_world_step_tests.rs`,
/// `healing_partial_solid.rs`).
const TOL: f64 = 1.0e-3;

fn corpus_root() -> Option<PathBuf> {
    std::env::var_os("MONSTERTRUCK_STEP_CORPUS")
        .map(PathBuf::from)
        .or_else(|| {
            std::env::var_os("HOME").map(|h| Path::new(&h).join("code/step-corpus/bigassy"))
        })
        .filter(|p| p.is_dir())
}

fn repo_fixtures() -> Vec<PathBuf> {
    let manifest = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut paths: Vec<PathBuf> = [
        manifest.join("../resources/step"),
        manifest.join("tests/fixtures/real-world"),
    ]
    .iter()
    .filter_map(|dir| std::fs::read_dir(dir).ok())
    .flatten()
    .filter_map(|entry| entry.ok().map(|e| e.path()))
    .filter(|p| {
        p.extension()
            .and_then(|e| e.to_str())
            .is_some_and(|e| e.eq_ignore_ascii_case("step") || e.eq_ignore_ascii_case("stp"))
    })
    .collect();
    paths.sort();
    paths
}

/// Per-chain digest sink. When `MT_STEP_UV_CLAMP_DIGEST_OUT` names a file, every
/// chain's produced points are hashed BIT-EXACTLY into one line, so two runs of
/// this harness across a code change diff to the exact set of chains that moved.
/// That is how U2 answers "what does a moved value do downstream" without a
/// behaviour-switching env var in production.
struct DigestSink(Option<std::io::BufWriter<std::fs::File>>);

impl DigestSink {
    fn open() -> Self {
        Self(
            // APPEND: each corpus file is its own test process, and they share
            // one digest file. Every line carries its file/solid/face key, so
            // the two runs being diffed are sorted first.
            std::env::var_os("MT_STEP_UV_CLAMP_DIGEST_OUT")
                .and_then(|path| {
                    std::fs::OpenOptions::new()
                        .create(true)
                        .append(true)
                        .open(path)
                        .ok()
                })
                .map(std::io::BufWriter::new),
        )
    }

    fn write(&mut self, key: &str, boundary: Option<&Vec<Point2>>) {
        use std::hash::{Hash, Hasher};
        use std::io::Write;
        let Some(out) = self.0.as_mut() else { return };
        let Some(points) = boundary else {
            let _ = writeln!(out, "{key} NONE");
            return;
        };
        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        for point in points {
            point.x.to_bits().hash(&mut hasher);
            point.y.to_bits().hash(&mut hasher);
        }
        let extent = points.iter().fold(
            (f64::MAX, f64::MIN, f64::MAX, f64::MIN),
            |(u0, u1, v0, v1), p| (u0.min(p.x), u1.max(p.x), v0.min(p.y), v1.max(p.y)),
        );
        let _ = writeln!(
            out,
            "{key} n={} digest={:016x} u=[{:.17e},{:.17e}] v=[{:.17e},{:.17e}]",
            points.len(),
            hasher.finish(),
            extent.0,
            extent.1,
            extent.2,
            extent.3,
        );
    }
}

/// Drives every (surface, boundary edge) pair of one shell through the sampled
/// projection. Returns the number of chains driven.
fn drive_shell(
    tag: &str,
    shell: &CompressedShell<monstertruck_geometry::prelude::Point3, Curve3D, Surface>,
    sink: &mut DigestSink,
) -> usize {
    let mut chains = 0;
    for (face_index, face) in shell.faces.iter().enumerate() {
        for (wire_index, wire) in face.boundaries.iter().enumerate() {
            for (edge_slot, edge_index) in wire.iter().enumerate() {
                let Some(edge) = shell.edges.get(edge_index.index) else {
                    continue;
                };
                let boundary = edge.curve.parameter_boundary_2d(&face.surface, TOL);
                sink.write(
                    &format!("{tag} f{face_index} w{wire_index} e{edge_slot}"),
                    boundary.as_ref(),
                );
                chains += 1;
            }
        }
    }
    chains
}

/// Loads `bytes` and drives at most `solid_cap` solids. Returns
/// `(solids driven, solids present, chains)`.
fn drive_bytes(
    tag: &str,
    bytes: &[u8],
    solid_cap: usize,
    sink: &mut DigestSink,
) -> (usize, usize, usize) {
    let Ok(table) = Table::from_step_bytes(bytes) else {
        return (0, 0, 0);
    };
    let present = table.manifold_solid_brep.len();
    let mut driven = 0;
    let mut chains = 0;
    // `Table::manifold_solid_brep` is a `HashMap`, so its iteration order is
    // randomized PER PROCESS. Sampling `take(cap)` off it drew a DIFFERENT set
    // of solids on every run -- measured: Ai-14R at cap = 4 gave 392 chains on
    // one run and 512 on the next. Sorting by entity id makes the sample a fact
    // about the file rather than about the run, which is what a before/after
    // digest diff needs.
    let mut ids: Vec<u64> = table.manifold_solid_brep.keys().copied().collect();
    ids.sort_unstable();
    for id in ids.into_iter().take(solid_cap) {
        let Ok(csolid) = table.to_compressed_solid(&table.manifold_solid_brep[&id]) else {
            continue;
        };
        driven += 1;
        for (boundary_index, shell) in csolid.boundaries.iter().enumerate() {
            chains += drive_shell(&format!("{tag} #{id} b{boundary_index}"), shell, sink);
        }
    }
    (driven, present, chains)
}

fn drive_path(path: &Path, solid_cap: usize, sink: &mut DigestSink) -> (usize, usize, usize) {
    let Ok(bytes) = std::fs::read(path) else {
        return (0, 0, 0);
    };
    let tag = path
        .file_name()
        .unwrap_or_default()
        .to_string_lossy()
        .into_owned();
    drive_bytes(&tag, &bytes, solid_cap, sink)
}

fn require_lens() -> bool {
    if uv_clamp::enabled() {
        return true;
    }
    eprintln!("SKIP: set MT_STEP_DEBUG_UV_CLAMP=1 to take the measurement.");
    false
}

/// Every in-repo STEP fixture, every solid.
#[test]
#[ignore = "measurement harness -- run with MT_STEP_DEBUG_UV_CLAMP=1 --no-capture"]
fn uv_clamp_census_over_in_repo_fixtures() {
    if !require_lens() {
        return;
    }
    uv_clamp::reset();
    let mut sink = DigestSink::open();
    let mut files = 0;
    let mut chains = 0;
    for path in repo_fixtures() {
        let (driven, present, file_chains) = drive_path(&path, usize::MAX, &mut sink);
        if driven == 0 && present == 0 {
            continue;
        }
        files += 1;
        chains += file_chains;
        eprintln!(
            "[uv-clamp] fixture {:<44} solids={driven}/{present} chains={file_chains}",
            path.file_name().unwrap_or_default().to_string_lossy(),
        );
    }
    eprintln!("[uv-clamp] fixtures: {files} files, {chains} chains");
    uv_clamp::snapshot().report("in-repo");
}

/// One corpus file, sampled by solid. Each file is its own test so each gets its
/// own 20-minute nextest budget and its own process -- which also means its own
/// thread-local census, so the per-file rows never bleed into each other.
///
/// The cap IS the sample and is printed on the result line;
/// `MT_STEP_UV_CLAMP_SOLID_CAP` overrides it for a deeper run, and
/// `MT_STEP_UV_CLAMP_DIGEST_OUT` names a per-run digest file.
fn corpus_file_census(name: &str, cap: usize) {
    if !require_lens() {
        return;
    }
    let Some(root) = corpus_root() else {
        eprintln!("SKIP: corpus absent.");
        return;
    };
    let cap = std::env::var("MT_STEP_UV_CLAMP_SOLID_CAP")
        .ok()
        .and_then(|v| v.parse::<usize>().ok())
        .unwrap_or(cap);
    if cap == 0 {
        // Scania-Engine: `Table::from_step_bytes` alone exceeds nextest's
        // 20-minute per-test kill (measured: the run times out at cap = 1
        // before a single chain is driven), so the default sample is EMPTY and
        // says so rather than reporting a killed run as a result. Opt in with
        // `MT_STEP_UV_CLAMP_SOLID_CAP=1` and a raised `--slow-timeout`.
        eprintln!("[uv-clamp] corpus {name:<44} NOT SAMPLED (cap 0)");
        return;
    }
    uv_clamp::reset();
    let mut sink = DigestSink::open();
    let started = std::time::Instant::now();
    let (driven, present, chains) = drive_path(&root.join(name), cap, &mut sink);
    eprintln!(
        "[uv-clamp] corpus {name:<44} solids={driven}/{present} (cap={cap}) chains={chains} \
         {:.1}s",
        started.elapsed().as_secs_f64(),
    );
    uv_clamp::snapshot().report(name);
}

macro_rules! corpus_census_tests {
    ($($test:ident => ($file:literal, $cap:expr),)*) => {$(
        #[test]
        #[ignore = "corpus measurement -- ~1 GB, run with MT_STEP_DEBUG_UV_CLAMP=1"]
        fn $test() { corpus_file_census($file, $cap); }
    )*};
}

corpus_census_tests! {
    uv_clamp_census_corpus_rotor => ("ROTOR-201NAL-Z7.STEP", usize::MAX),
    uv_clamp_census_corpus_rocky_house => ("Rocky_House.stp", 12),
    uv_clamp_census_corpus_cruise_assembly => ("Cruise_Assembly.stp", 12),
    uv_clamp_census_corpus_umc500 => ("UMC-500_SS_Solid_Model_2019-06_r1.stp", 12),
    uv_clamp_census_corpus_ai14r => ("Ai-14R.stp", 4),
    uv_clamp_census_corpus_nissan => ("NissanGT-R.STEP", 4),
    uv_clamp_census_corpus_scania_8x4 => ("Scania-8x4.stp", 4),
    uv_clamp_census_corpus_scania_engine => ("Scania-Engine-V8-XT-Turbo.step", 0),
}
