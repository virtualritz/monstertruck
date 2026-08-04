//! STEP-export watertightness invariant.
//!
//! In a closed, oriented, manifold B-rep every edge is shared by exactly two
//! faces, so each `EDGE_CURVE` entity must be referenced by exactly two
//! `ORIENTED_EDGE` entities in the emitted file. An edge referenced once is a
//! naked (boundary) edge -- the shell is not closed; an edge referenced three
//! or more times is non-manifold; an `ORIENTED_EDGE` pointing at an undefined
//! `EDGE_CURVE` is a dangling reference. This guards the *serialization*, which
//! the in-memory [`ShellCondition::Closed`] check cannot.
//!
//! This is a clean-room reimplementation of a standard ISO 10303-21 / B-rep
//! invariant; no third-party code is used.

use monstertruck_modeling::*;
use monstertruck_step::save::*;
use std::collections::{BTreeMap, BTreeSet};

macro_rules! dir ( () => { concat!(env!("CARGO_MANIFEST_DIR"), "/../resources/shape/") });

const SOLID_JSONS: &[&str] = &[
    concat!(dir!(), "bottle.json"),
    concat!(dir!(), "punched-cube.json"),
    concat!(dir!(), "torus-punched-cube.json"),
    concat!(dir!(), "cube-in-cube.json"),
];

/// Splits a STEP data section into entity chunks and returns, for each chunk
/// that starts with `#<id> = <TYPE>(`, the tuple `(id, type, args)`.
fn parse_entity(chunk: &str) -> Option<(u64, &str, &str)> {
    let chunk = chunk.trim();
    let rest = chunk.strip_prefix('#')?;
    let eq = rest.find('=')?;
    let id = rest[..eq].trim().parse().ok()?;
    let after = rest[eq + 1..].trim_start();
    // Complex entities start with `(` (no leading type name); their `type`
    // slice is empty, so they never match `EDGE_CURVE` / `ORIENTED_EDGE`.
    let paren = after.find('(')?;
    Some((id, after[..paren].trim(), &after[paren..]))
}

/// Extracts the first `#<id>` reference inside an argument list. An
/// `ORIENTED_EDGE` references exactly one entity (its `EDGE_CURVE`).
fn first_reference(args: &str) -> Option<u64> {
    let hash = args.find('#')?;
    let after = &args[hash + 1..];
    let end = after
        .find(|c: char| !c.is_ascii_digit())
        .unwrap_or(after.len());
    after[..end].parse().ok()
}

/// Returns the set of defined `EDGE_CURVE` ids and the per-id count of
/// `ORIENTED_EDGE` references.
fn edge_uses(step: &str) -> (BTreeSet<u64>, BTreeMap<u64, usize>) {
    let mut defined = BTreeSet::new();
    let mut references: BTreeMap<u64, usize> = BTreeMap::new();
    for chunk in step.split(';') {
        let Some((id, ty, args)) = parse_entity(chunk) else {
            continue;
        };
        match ty {
            "EDGE_CURVE" => {
                defined.insert(id);
            }
            "ORIENTED_EDGE" => {
                if let Some(edge) = first_reference(args) {
                    *references.entry(edge).or_insert(0) += 1;
                }
            }
            _ => {}
        }
    }
    (defined, references)
}

/// Parses `step` and asserts it describes a closed manifold: it is
/// syntactically valid, every `EDGE_CURVE` is referenced by exactly two
/// `ORIENTED_EDGE`s, and no `ORIENTED_EDGE` references an undefined edge.
fn assert_closed_manifold(label: &str, step: &str) -> anyhow::Result<()> {
    step_p21::parser::parse(step)
        .map_err(|error| anyhow::anyhow!("{label}: emitted STEP did not parse: {error}"))?;

    let (defined, references) = edge_uses(step);
    let mut problems = Vec::new();
    for id in &defined {
        match references.get(id).copied().unwrap_or(0) {
            2 => {}
            0 => problems.push(format!("#{id}: EDGE_CURVE never referenced (unused edge)")),
            1 => problems.push(format!("#{id}: EDGE_CURVE referenced once (naked edge)")),
            n => problems.push(format!(
                "#{id}: EDGE_CURVE referenced {n} times (non-manifold)"
            )),
        }
    }
    for id in references.keys() {
        if !defined.contains(id) {
            problems.push(format!(
                "#{id}: ORIENTED_EDGE references an undefined EDGE_CURVE (dangling)"
            ));
        }
    }

    anyhow::ensure!(
        problems.is_empty(),
        "{label}: STEP output is not a closed manifold:\n  {}",
        problems.join("\n  ")
    );
    Ok(())
}

#[test]
fn solid_fixtures_export_closed_manifolds() -> anyhow::Result<()> {
    for json_file in SOLID_JSONS {
        let json = std::fs::read(json_file)?;
        let solid: CompressedSolid = serde_json::from_reader(json.as_slice())?;
        let step =
            CompleteStepDisplay::new(StepModel::from(&solid), Default::default()).to_string();

        assert_closed_manifold(json_file, &step)?;
    }
    Ok(())
}
