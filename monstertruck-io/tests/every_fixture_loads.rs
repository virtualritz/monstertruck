//! Every STEP fixture in the repository must load. No exceptions, no list to
//! forget to update.
//!
//! This exists because of a real failure. Folding `monstertruck-step` into this
//! crate moved `tests/fixtures/real-world/`, and seven hardcoded
//! `../monstertruck-step/tests/fixtures/real-world` paths elsewhere in the
//! workspace were left behind. Nothing here noticed: every fixture test names
//! its files as string literals, so a fixture that has MOVED is
//! indistinguishable from one that was never mentioned. The breakage surfaced
//! far away, as eight unrelated-looking boolean rows dying at `read`, and cost
//! another agent a debugging session to trace back.
//!
//! Two properties make this row catch that class of defect:
//!
//! 1. **The corpora are ENUMERATED, not listed.** A fixture added to either
//!    directory is covered the moment it lands, with nothing to update here.
//! 2. **An empty or missing directory FAILS.** This is the whole point. A glob
//!    over a directory that moved yields zero files, and a test that merely
//!    iterates would report success over nothing -- the precise shape of the
//!    original defect. Each corpus therefore asserts a floor on how many files
//!    it must contain.
//!
//! Every failure is collected and reported together: when a loader change
//! breaks input handling, seeing every affected file at once is worth far more
//! than seeing whichever one sorted first.

#![cfg(feature = "load")]

use monstertruck_io::step::load::Table;
use std::path::{Path, PathBuf};

/// A directory of fixtures, and the minimum number of files it must hold.
///
/// The floor is a MOVED-DIRECTORY GUARD, not an inventory count. It is set below
/// the current population on purpose, so adding a fixture never fails this row
/// while removing the directory always does.
struct Corpus {
    /// Path relative to this crate's manifest directory.
    relative_path: &'static str,
    /// Fewer files than this means the directory moved, emptied, or was gated
    /// out -- all of which must be loud.
    minimum_files: usize,
}

const CORPORA: [Corpus; 1] = [
    // The shared `resources` submodule: small analytic solids plus ABC samples.
    // The frozen third-party exports live only in the private tree, so this
    // repository has one corpus where that one has two.
    Corpus {
        relative_path: "../resources/step",
        minimum_files: 10,
    },
];

fn corpus_dir(corpus: &Corpus) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join(corpus.relative_path)
}

/// Every `.step`/`.stp` file in `dir`, sorted so failure output is stable.
fn step_files(dir: &Path) -> Vec<PathBuf> {
    let entries = std::fs::read_dir(dir)
        .unwrap_or_else(|err| panic!("fixture directory {} is unreadable: {err}", dir.display()));
    let mut files: Vec<PathBuf> = entries
        .filter_map(|entry| entry.ok().map(|e| e.path()))
        .filter(|path| {
            path.extension()
                .and_then(|ext| ext.to_str())
                .is_some_and(|ext| {
                    ext.eq_ignore_ascii_case("step") || ext.eq_ignore_ascii_case("stp")
                })
        })
        .collect();
    files.sort();
    files
}

#[test]
fn every_fixture_in_the_repository_loads() {
    let mut failures: Vec<String> = Vec::new();
    let mut loaded = 0usize;

    for corpus in &CORPORA {
        let dir = corpus_dir(corpus);
        assert!(
            dir.is_dir(),
            "fixture directory {} does not exist -- it moved, or a rename left this path behind",
            dir.display(),
        );

        let files = step_files(&dir);
        assert!(
            files.len() >= corpus.minimum_files,
            "{} holds {} STEP files, expected at least {}. A directory that moved \
             or emptied yields nothing to iterate, and a sweep over nothing passes \
             silently -- which is the defect this row exists to catch.",
            dir.display(),
            files.len(),
            corpus.minimum_files,
        );

        for path in files {
            let name = path
                .file_name()
                .unwrap_or_default()
                .to_string_lossy()
                .into_owned();
            let bytes = match std::fs::read(&path) {
                Ok(bytes) => bytes,
                Err(err) => {
                    failures.push(format!("{name}: unreadable: {err}"));
                    continue;
                }
            };
            match Table::from_step_bytes(&bytes) {
                Ok(table) => {
                    // Parsing "succeeded" is not enough: a file that yields no
                    // geometry at all has not been read in any useful sense, and
                    // every fixture here carries points.
                    if table.cartesian_point.is_empty() {
                        failures.push(format!(
                            "{name}: parsed, but produced ZERO cartesian points -- \
                             read as empty rather than read"
                        ));
                    } else {
                        loaded += 1;
                    }
                }
                Err(err) => failures.push(format!("{name}: {err}")),
            }
        }
    }

    assert!(
        failures.is_empty(),
        "{} of {} fixtures failed to load:\n  {}",
        failures.len(),
        failures.len() + loaded,
        failures.join("\n  "),
    );
    assert!(
        loaded >= 10,
        "expected at least 10 fixtures (10 present when this was written), loaded {loaded}"
    );
}
