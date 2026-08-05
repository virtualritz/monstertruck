//! Load-reachability census over the big-assembly corpus.
//!
//! Answers one question: which real CAD exports can the loader read at all?
//! Corpus-gated and `#[ignore]`d -- ~1 GB of input, far too slow for the default
//! gate. Run with `--ignored --nocapture`.

use monstertruck_io::step::load::Table;

const FILES: [&str; 8] = [
    "ROTOR-201NAL-Z7.STEP",
    "Rocky_House.stp",
    "Cruise_Assembly.stp",
    "UMC-500_SS_Solid_Model_2019-06_r1.stp",
    "Ai-14R.stp",
    "NissanGT-R.STEP",
    "Scania-8x4.stp",
    "Scania-Engine-V8-XT-Turbo.step",
];

fn corpus_root() -> Option<std::path::PathBuf> {
    std::env::var_os("MONSTERTRUCK_STEP_CORPUS")
        .map(std::path::PathBuf::from)
        .or_else(|| {
            std::env::var_os("HOME")
                .map(|h| std::path::Path::new(&h).join("code/step-corpus/bigassy"))
        })
        .filter(|p| p.is_dir())
}

#[test]
#[ignore = "corpus census -- ~1 GB, run explicitly"]
fn corpus_load_census() {
    let Some(root) = corpus_root() else {
        eprintln!("SKIP: corpus absent; run fixtures/step-bigassy/fetch.sh");
        return;
    };
    let mut ok = 0;
    let mut failed = 0;
    for name in FILES {
        let path = root.join(name);
        let Ok(bytes) = std::fs::read(&path) else {
            eprintln!("  {name:<40} ABSENT");
            continue;
        };
        let mb = bytes.len() / 1_048_576;
        let started = std::time::Instant::now();
        match Table::from_step_bytes(&bytes) {
            Ok(t) => {
                ok += 1;
                eprintln!(
                    "  {name:<40} {mb:>4} MB  OK    solids={:<5} {:.1}s",
                    t.manifold_solid_brep.len(),
                    started.elapsed().as_secs_f64(),
                );
            }
            Err(e) => {
                failed += 1;
                let msg = format!("{e:?}");
                let first = msg
                    .lines()
                    .next()
                    .unwrap_or("")
                    .chars()
                    .take(72)
                    .collect::<String>();
                eprintln!("  {name:<40} {mb:>4} MB  FAIL  {first}");
            }
        }
    }
    eprintln!(
        "CENSUS: {ok} loaded, {failed} refused, of {} files",
        FILES.len()
    );
}

/// The two refusals, in full: where does the tokenizer stop, and what is there?
#[test]
#[ignore = "corpus diagnostic"]
fn refused_files_failure_points() {
    let Some(root) = corpus_root() else { return };
    for name in ["UMC-500_SS_Solid_Model_2019-06_r1.stp", "Ai-14R.stp"] {
        let Ok(bytes) = std::fs::read(root.join(name)) else {
            continue;
        };
        if let Err(e) = Table::from_step_bytes(&bytes) {
            eprintln!("=== {name}\n{e:?}\n");
        }
    }
}
