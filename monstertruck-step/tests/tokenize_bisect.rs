//! Diagnostic: which construct does the tokenizer refuse?
//!
//! Two big-assembly files are refused with an identical anchored error whose
//! position is NOT the failure site (nom reports the furthest tag attempted).
//! This binary-searches the data section by BYTE OFFSET -- cut at a `;` line
//! boundary, close the file properly, parse -- to find the first prefix that
//! fails. Byte offsets rather than entity counts, because entities span lines
//! and a naive splitter mis-cuts them.
//!
//! `#[ignore]`d and corpus-gated.

use monstertruck_step::load::Table;

fn corpus(name: &str) -> Option<std::path::PathBuf> {
    let root = std::env::var_os("MONSTERTRUCK_STEP_CORPUS")
        .map(std::path::PathBuf::from)
        .or_else(|| {
            std::env::var_os("HOME")
                .map(|h| std::path::Path::new(&h).join("code/step-corpus/bigassy"))
        })?;
    let p = root.join(name);
    p.is_file().then_some(p)
}

/// Largest `;`-terminated line boundary at or before `at`.
fn cut_point(body: &str, at: usize) -> usize {
    body[..at.min(body.len())]
        .rfind(";\r\n")
        .or_else(|| body[..at.min(body.len())].rfind(";\n"))
        .map_or(0, |i| i + 1)
}

fn parses(header: &str, body: &str, cut: usize) -> bool {
    let text = format!("{header}\n{}\nENDSEC;\nEND-ISO-10303-21;\n", &body[..cut]);
    Table::from_step(&text).is_ok()
}

fn bisect(label: &str, name: &str) {
    let Some(path) = corpus(name) else {
        eprintln!("SKIP {label}: absent");
        return;
    };
    let raw = std::fs::read(&path).expect("readable");
    let text: String = raw.iter().map(|&b| b as char).collect();
    let di = text.find("DATA;").expect("has a DATA section") + "DATA;".len();
    let (header, body) = text.split_at(di);
    let body = body.trim_end_matches(|c: char| c.is_whitespace());
    let body = body.strip_suffix("END-ISO-10303-21;").unwrap_or(body);
    let body = body.trim_end();
    let body = body.strip_suffix("ENDSEC;").unwrap_or(body);

    if parses(header, body, body.len()) {
        eprintln!("{label}: WHOLE BODY PARSES -- the refusal is in the trailer, not the data");
        return;
    }
    // invariant: lo parses, hi does not
    let (mut lo, mut hi) = (0usize, body.len());
    while hi - lo > 400 {
        let mid = cut_point(body, lo + (hi - lo) / 2);
        if mid <= lo || mid >= hi {
            break;
        }
        if parses(header, body, mid) {
            lo = mid
        } else {
            hi = mid
        }
    }
    let start = body[..lo]
        .rfind(";\r\n")
        .map_or(lo.saturating_sub(200), |i| i + 3);
    eprintln!(
        "{label}: first failing region at byte {lo}..{hi} of {}\n----\n{}\n----",
        body.len(),
        &body[start..hi.min(start + 600)],
    );
}

#[test]
#[ignore = "tokenizer bisection probe"]
fn bisect_umc500() { bisect("UMC-500", "UMC-500_SS_Solid_Model_2019-06_r1.stp") }

#[test]
#[ignore = "tokenizer bisection probe"]
fn bisect_ai14r() { bisect("Ai-14R", "Ai-14R.stp") }

/// Minimal reproduction: a part-21 string containing an ESCAPED APOSTROPHE.
///
/// ISO 10303-21 escapes a literal `'` inside a string by doubling it. Imperial
/// CAD is full of them -- inch marks in thread callouts and part names.
///
/// Published step_p21 0.4.0 refuses the escape. UN-IGNORED: this test IS the
/// acceptance test for the upstream fix, which landed in
/// <https://github.com/ricosjp/step_p21/pull/254> (originally
/// <https://github.com/ricosjp/step_p21/pull/251>) and reaches us through the
/// `[patch.crates-io]` rev pin in the workspace manifest. It goes red again if
/// that pin is dropped before step_p21 publishes a release containing the fix.
#[test]
fn escaped_apostrophe_in_a_string_literal() {
    let base = |name: &str| {
        format!(
            "ISO-10303-21;\nHEADER;\n\
             FILE_DESCRIPTION(('x'),'2;1');\n\
             FILE_NAME('x','2019-01-01T00:00:00',('x'),('x'),'x','x','x');\n\
             FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n\
             #1 = CARTESIAN_POINT ( '{name}', ( 0.0, 0.0, 0.0 ) ) ;\n\
             ENDSEC;\nEND-ISO-10303-21;\n"
        )
    };

    // Control: a plain name parses, so the harness itself is sound.
    assert!(
        Table::from_step(&base("PLAIN")).is_ok(),
        "control: a plain string literal must parse",
    );

    // The construct from UMC-500 line #671342: `.330X90''` is `.330X90` + `'`.
    let doubled = base("5/16-18 UNC-2B X.9 DPCHAMFER .330X90''");
    let result = Table::from_step(&doubled);
    assert!(
        result.is_ok(),
        "ISO 10303-21 escapes a literal apostrophe by doubling it, and imperial \
         CAD emits these constantly (inch marks in thread callouts). Refusing \
         them costs whole assemblies: this is the sole reason \
         UMC-500_SS_Solid_Model_2019-06_r1.stp is unreadable. Got: {:?}",
        result.err(),
    );
}

fn minimal(entity: &str) -> String {
    format!(
        "ISO-10303-21;\nHEADER;\n\
         FILE_DESCRIPTION(('x'),'2;1');\n\
         FILE_NAME('x','2019-01-01T00:00:00',('x'),('x'),'x','x','x');\n\
         FILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));\nENDSEC;\nDATA;\n\
         {entity}\nENDSEC;\nEND-ISO-10303-21;\n"
    )
}

/// An EMPTY AGGREGATE `()` must tokenize.
///
/// ISO 10303-21 permits an empty list, and exporters emit them -- Ai-14R.stp has
/// 41, e.g. `ADVANCED_FACE('',(),#57075,.T.)`. Refusing them costs the whole
/// file, and this is the sole reason that 99 MB assembly is unreadable.
///
/// The other constructs in the same bisected window are asserted alongside as
/// CONTROLS: they parse, so the failure is the empty aggregate specifically and
/// not the window, the entity type, or the harness. In particular the non-empty
/// aggregate differs by exactly one element.
///
/// Published step_p21 0.4.0 refuses `()`. UN-IGNORED: this test IS the
/// acceptance test for the upstream fix, which landed in
/// <https://github.com/ricosjp/step_p21/pull/254> (originally
/// <https://github.com/ricosjp/step_p21/pull/251>) and reaches us through the
/// `[patch.crates-io]` rev pin in the workspace manifest.
#[test]
fn empty_aggregate_in_an_entity_parameter() {
    let cases = [
        ("empty aggregate", "#1=ADVANCED_FACE('',(),#2,.T.);"),
        (
            "non-empty aggregate (control)",
            "#1=ADVANCED_FACE('',(#3),#2,.T.);",
        ),
        (
            "nested typed param",
            "#1=POINT_STYLE('',MARKER_TYPE(.DOT.),POSITIVE_LENGTH_MEASURE(0.1),#2);",
        ),
        (
            "multi-line aggregate",
            "#1=GEOMETRIC_CURVE_SET('',(#5,#6,\n#7));",
        ),
        ("plain control", "#1=CARTESIAN_POINT('',(0.0,0.0,0.0));"),
    ];
    for (label, entity) in cases {
        let result = Table::from_step(&minimal(entity));
        if label == "empty aggregate" {
            assert!(
                result.is_ok(),
                "ISO 10303-21 permits an empty aggregate and Ai-14R.stp has 41 of \
                 them; refusing one costs the entire 99 MB file. Got: {:?}",
                result.err(),
            );
        } else {
            assert!(
                result.is_ok(),
                "control `{label}` must parse -- if it does not, the failure is \
                 broader than the empty aggregate and this test is measuring the \
                 wrong thing: {:?}",
                result.err(),
            );
        }
    }
}
