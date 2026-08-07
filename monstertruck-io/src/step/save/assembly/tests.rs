//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

use super::*;
use monstertruck_modeling::base::One;

// Lightweight `StepFormat` model used only by these tests: emits a
// sentinel `MOCK_SHAPE` entity so we can detect that nodes' shapes
// were rendered without depending on the heavier `StepModel` apparatus.
#[derive(Clone, Debug)]
struct MockShape;

impl StepFormat for MockShape {
    fn fmt(&self, idx: usize, f: &mut Formatter<'_>) -> Result {
        writeln!(f, "#{idx} = MOCK_SHAPE();")
    }
}

impl StepLength for MockShape {
    fn step_length(&self) -> usize { 1 }
}

#[test]
fn step_design_emits_application_context_and_per_node_entities() -> anyhow::Result<()> {
    let design: StepDesign<MockShape, Option<MockShape>, Matrix4> =
        StepDesign::from_model(MockShape);
    let output = format!("{design}");

    assert!(
        output.contains("APPLICATION_CONTEXT('generated shape data')"),
        "default application context should appear in the output. \
         actual output:\n{output}",
    );
    assert!(
        output.contains("GEOMETRIC_REPRESENTATION_CONTEXT(3)"),
        "representation context preamble should appear in the output.",
    );
    assert!(
        output.contains("AXIS2_PLACEMENT_3D('',"),
        "the global identity matrix should be emitted as an AXIS2_PLACEMENT_3D.",
    );
    assert!(
        output.contains("SHAPE_DEFINITION_REPRESENTATION("),
        "the single node should produce a SHAPE_DEFINITION_REPRESENTATION.",
    );
    assert!(
        output.contains("MOCK_SHAPE();"),
        "the node's shape entity should be rendered.",
    );
    Ok(())
}

#[test]
fn step_design_with_custom_application_context_round_trips() -> anyhow::Result<()> {
    let mut assembly: Assembly<Option<MockShape>, PartAttributes, Matrix4, PartAttributes> =
        Assembly::new();
    assembly.create_node(NodeEntity {
        shape: Some(MockShape),
        attrs: PartAttributes {
            id: "PART-0001".to_owned(),
            name: "Mock part".to_owned(),
            description: "Single mock part used in a structural round-trip test.".to_owned(),
            shape_representation: None,
        },
    });
    let design: StepDesign<MockShape, _, _> =
        StepDesign::with_application_context(assembly, "monstertruck-step test".to_owned());
    let output = format!("{design}");

    assert!(
        output.contains("APPLICATION_CONTEXT('monstertruck-step test')"),
        "custom application context should appear in the output.",
    );
    assert!(
        output.contains("PRODUCT('PART-0001', 'Mock part',"),
        "the PRODUCT entity should embed the supplied id and name. actual output:\n{output}",
    );
    Ok(())
}

// Shape stub used by the round-trip test. The assembly nodes all use
// `shape: None`, so neither method ever runs; they only exist to
// satisfy the `Model: StepFormat + StepLength` trait bounds.
#[derive(Clone, Debug)]
struct NeverShape;

impl StepFormat for NeverShape {
    fn fmt(&self, _: usize, _: &mut Formatter<'_>) -> Result {
        unreachable!("NeverShape::fmt should not be called; nodes use `shape: None`.")
    }
}

impl StepLength for NeverShape {
    fn step_length(&self) -> usize {
        unreachable!("NeverShape::step_length should not be called; nodes use `shape: None`.")
    }
}

/// Builds an assembly with three nodes (A, B, C) and two edges
/// (A -> B, A -> C) sharing the identity transform, formats it as
/// STEP via [`CompleteStepDisplay`], re-parses the result with
/// [`crate::step::load::Table::from_step`], and verifies that the
/// re-parsed assembly has the same node count, the same set of
/// [`PartAttributes`], and the same out-degree per node.
#[test]
fn assembly_round_trip_preserves_topology_and_attributes() -> anyhow::Result<()> {
    use crate::step::load::Table;
    use crate::step::save::{CompleteStepDisplay, StepHeaderDescriptor};

    let mut assembly: Assembly<Option<NeverShape>, PartAttributes, Matrix4, PartAttributes> =
        Assembly::new();
    let parts = ["A", "B", "C"].map(|name| {
        assembly.create_node(NodeEntity {
            shape: None,
            attrs: PartAttributes {
                id: format!("PART-{name}"),
                name: format!("Part {name}"),
                description: format!("test part {name}"),
                // An assembly built in memory has no file to be an id into.
                shape_representation: None,
            },
        })
    });
    let edge_attributes = ["AB", "AC"].map(|name| PartAttributes {
        id: format!("LINK-{name}"),
        name: format!("Link {name}"),
        description: format!("test link {name}"),
        shape_representation: None,
    });
    for (target_index, (target, attrs)) in [parts[1], parts[2]]
        .iter()
        .zip(edge_attributes.iter())
        .enumerate()
    {
        assembly.create_edge(
            parts[0],
            *target,
            EdgeEntity {
                matrix: Matrix4::one(),
                attrs: attrs.clone(),
            },
        );
        let _ = target_index; // suppress unused-variable warning on naming.
    }
    let design: StepDesign<NeverShape, _, _> = StepDesign::new(assembly);

    let step_string = format!(
        "{}",
        CompleteStepDisplay::new(design, StepHeaderDescriptor::default())
    );

    let table = Table::from_step(&step_string)?;
    let round_trip = table.step_assy()?;
    assert_eq!(
        round_trip.len(),
        3,
        "round-tripped assembly should have three nodes."
    );

    let original_ids: std::collections::HashSet<&str> =
        ["PART-A", "PART-B", "PART-C"].into_iter().collect();
    let parsed_ids: std::collections::HashSet<&str> = round_trip
        .all_nodes()
        .map(|node| node.attributes().id.as_str())
        .collect();
    assert_eq!(
        parsed_ids, original_ids,
        "round-trip should preserve every node's `PartAttributes::id`. \
         Parsed STEP:\n{step_string}",
    );

    let total_edges: usize = round_trip
        .all_nodes()
        .map(|node| node.edges().count())
        .sum();
    assert_eq!(
        total_edges, 2,
        "round-tripped assembly should have two edges."
    );

    let root = round_trip
        .all_nodes()
        .find(|node| node.attributes().id == "PART-A")
        .expect("the parsed assembly should contain `PART-A`.");
    assert_eq!(
        root.edges().count(),
        2,
        "the root node `PART-A` should retain both outgoing edges.",
    );

    Ok(())
}
