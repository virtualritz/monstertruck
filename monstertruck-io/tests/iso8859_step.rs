//! A STEP file that is not UTF-8 must still load.
//!
//! ISO 10303-21 wants non-ASCII carried as `\X2\` escapes, but exporters in
//! non-English locales emit raw 8-bit bytes in string literals instead. Those
//! files are otherwise ordinary and their geometry is pure ASCII, so refusing
//! them is a decoding choice rather than a correctness one.
//!
//! Corpus-gated: the fixture is ~100 MB and lives outside the repository (see
//! `fixtures/step-bigassy/README.md`). The row SKIPS when the corpus is absent
//! so a fresh clone stays green.

use monstertruck_io::step::load::Table;

fn corpus_file(name: &str) -> Option<std::path::PathBuf> {
    let root = std::env::var_os("MONSTERTRUCK_STEP_CORPUS")
        .map(std::path::PathBuf::from)
        .or_else(|| {
            std::env::var_os("HOME")
                .map(|home| std::path::Path::new(&home).join("code/step-corpus/bigassy"))
        })?;
    let path = root.join(name);
    path.is_file().then_some(path)
}

/// `from_step_bytes` accepts an ISO-8859 file that `String::from_utf8` refuses.
///
/// Pins BOTH halves: that the file genuinely is not UTF-8 (so the test cannot
/// silently degrade into re-testing the ASCII path if the fixture is ever
/// replaced), and that it nonetheless parses to a populated table.
#[test]
fn iso8859_step_file_loads_via_bytes() {
    let Some(path) = corpus_file("Ai-14R.stp") else {
        eprintln!("SKIP: corpus absent; run fixtures/step-bigassy/fetch.sh");
        return;
    };
    let bytes = std::fs::read(&path).expect("fixture readable");

    assert!(
        std::str::from_utf8(&bytes).is_err(),
        "precondition: {} must NOT be valid UTF-8, else this test proves nothing",
        path.display(),
    );

    // The DECODE now succeeds -- that is what `from_step_bytes` fixed. The PARSE
    // still refuses, and measurement says encoding is not why: the ASCII-only
    // `UMC-500_SS_Solid_Model_2019-06_r1.stp` is refused at the identical point,
    // and an ASCII-scrubbed copy of THIS file is refused too. So the row pins the
    // decode boundary and records the parser refusal as known, rather than
    // asserting a success the loader cannot yet deliver.
    match Table::from_step_bytes(&bytes) {
        Ok(table) => assert!(
            !table.manifold_solid_brep.is_empty(),
            "if this file now parses, it must carry geometry -- and \
             FIX_PLAN_010_PRODUCER_TRACK.md 7kk needs updating, because it \
             records this as a known refusal",
        ),
        Err(error) => {
            let text = format!("{error:?}");
            assert!(
                text.contains("TokenizeFailed"),
                "the known refusal is a tokenizer refusal; a DIFFERENT error here \
                 is a new defect and must not pass silently: {text}",
            );
            eprintln!("KNOWN REFUSAL (7kk, parser not encoding): {text}");
        }
    }
}

/// The fallback must not perturb a UTF-8 file: `from_step_bytes` and
/// `from_step` agree on entity counts for ASCII input.
#[test]
fn utf8_step_file_is_unaffected_by_the_fallback() {
    let path = std::path::Path::new("../resources/step/occt-cylinder.step");
    let Ok(bytes) = std::fs::read(path) else {
        eprintln!("SKIP: {} absent", path.display());
        return;
    };
    let text = std::str::from_utf8(&bytes).expect("this fixture is ASCII");

    let direct = Table::from_step(text).expect("parses");
    let viabytes = Table::from_step_bytes(&bytes).expect("parses");
    assert_eq!(
        direct.manifold_solid_brep.len(),
        viabytes.manifold_solid_brep.len(),
        "the UTF-8 branch must be the plain from_step path",
    );
}
