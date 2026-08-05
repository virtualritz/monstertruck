//! STEP serialisation of [`monstertruck_assembly`] graphs.
//!
//! Emits a complete STEP product structure -- application context, units,
//! identity transform, plus one `SHAPE_DEFINITION_REPRESENTATION` chain per
//! assembly node and one `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` chain per
//! assembly edge -- from an [`Assembly`] whose node payload is a sequence of
//! [`StepFormat`]-displayable shapes and whose edge payload is a transform
//! matrix.
//!
//! The top-level entry point is [`StepDesign`], which is built from an
//! [`Assembly`] or from a single shape via [`StepDesign::from_model`] and
//! implements [`std::fmt::Display`] to produce the STEP data section.

use super::{Result, *};
use crate::step::common::PartAttributes;
use monstertruck_assembly::assy::*;
use monstertruck_modeling::Matrix4;

/// Index of the `APPLICATION_CONTEXT` entity, always written first.
const GLOBAL_APPLICATION_CONTEXT_INDEX: usize = 1;
/// Index of the shared `GEOMETRIC_REPRESENTATION_CONTEXT` block.
const COMMON_REPRESENTATION_CONTEXT_INDEX: usize = 2;
/// Index of the identity transform that every node's shape references.
const GLOBAL_IDENTITY_MATRIX: usize =
    COMMON_REPRESENTATION_CONTEXT_INDEX + MonstertruckRepresentationContext::LENGTH;

/// Representation-context preamble used by every emitted STEP file: a
/// three-dimensional geometric context whose length unit and distance
/// accuracy come from the wrapped [`StepMeasurementContext`] (millimetre
/// lengths and a `1.0E-6` tolerance by default), with radian angles and
/// steradian solid angles.
#[derive(Clone, Copy, Debug)]
struct MonstertruckRepresentationContext(StepMeasurementContext);

impl StepFormat for MonstertruckRepresentationContext {
    fn fmt(&self, idx: usize, f: &mut Formatter<'_>) -> Result {
        let context_idx = idx;
        let length_unit_idx = idx + 1;
        let plane_angle_unit_idx = idx + 2;
        let solid_angle_unit_idx = idx + 3;
        let tolerance_idx = idx + 4;
        let length_prefix = self.0.length_prefix;
        let accuracy = self.0.accuracy();
        f.write_fmt(format_args!(
"#{context_idx} = (
    GEOMETRIC_REPRESENTATION_CONTEXT(3)
    GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{tolerance_idx}))
    GLOBAL_UNIT_ASSIGNED_CONTEXT((#{length_unit_idx}, #{plane_angle_unit_idx}, #{solid_angle_unit_idx}))
    REPRESENTATION_CONTEXT('Context #1', '3D Context with UNIT and UNCERTAINTY')
);
#{length_unit_idx} = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT({length_prefix},.METRE.));
#{plane_angle_unit_idx} = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );
#{solid_angle_unit_idx} = ( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() );
#{tolerance_idx} = UNCERTAINTY_MEASURE_WITH_UNIT({accuracy}, \
#{length_unit_idx}, 'distance_accuracy_value', 'confusion accuracy');\n"
        ))
    }
}
impl_const_step_length!(MonstertruckRepresentationContext, 5);

// Heterogeneous payload used while emitting a node's `SHAPE_REPRESENTATION`:
// each entry is either a per-edge transform matrix (one of the node's
// outgoing edges) or one of the node's shapes. Sharing a single
// `StepFormat` impl across both lets the emit loop be uniform.
#[derive(Clone, Debug)]
enum MatrixOrModel<Matrix, Model> {
    Matrix(Matrix),
    Model(Model),
}

impl<Matrix, Model> StepFormat for MatrixOrModel<Matrix, Model>
where
    Model: StepFormat + StepLength,
    Matrix: StepFormat + ConstStepLength,
{
    fn fmt(&self, idx: usize, f: &mut Formatter<'_>) -> Result {
        match self {
            Self::Matrix(mat) => StepFormat::fmt(mat, idx, f),
            Self::Model(model) => StepFormat::fmt(model, idx, f),
        }
    }
}

impl<'a, Model, Models, Matrix> StepFormat
    for Node<'a, NodeEntity<Models, PartAttributes>, EdgeEntity<Matrix, PartAttributes>>
where
    Model: StepFormat + StepLength,
    for<'b> &'b Models: IntoIterator<Item = &'b Model>,
    Matrix: Copy,
    MatrixAsAxis<Matrix>: StepFormat + ConstStepLength,
{
    fn fmt(&self, idx: usize, f: &mut Formatter<'_>) -> Result {
        let NodeEntity {
            shape,
            attrs:
                PartAttributes {
                    id,
                    name,
                    description,
                    // DELIBERATELY DROPPED, and destructured explicitly so a
                    // future field cannot be dropped by accident. A load-side
                    // `SHAPE_REPRESENTATION` id indexes the file it was read
                    // out of; this emitter allocates its own numbering (the
                    // `sr_idx` below), so writing the carried id back would
                    // emit a dangling reference. See
                    // `PartAttributes::shape_representation`.
                    shape_representation: _,
                },
        } = self.entity();
        let sdr_idx = idx;
        let pds_idx = idx + 1;
        let pd_idx = idx + 2;
        let pdf_idx = idx + 3;
        let p_idx = idx + 4;
        let pdc_idx = idx + 5;
        let pc_idx = idx + 6;
        let sr_idx = idx + 7;

        let mut shape_indices = vec![GLOBAL_IDENTITY_MATRIX];
        let mut cursor = idx + 8;
        let mut displays = Vec::new();
        for edge in self.edges() {
            let mat = MatrixAsAxis(*edge.matrix());
            shape_indices.push(cursor);
            displays.push(StepDisplay::new(MatrixOrModel::Matrix(mat), cursor));
            cursor += MatrixAsAxis::<Matrix>::LENGTH;
        }
        for shape in shape {
            shape_indices.push(cursor);
            displays.push(StepDisplay::new(MatrixOrModel::Model(shape), cursor));
            cursor += shape.step_length();
        }

        let shape_indices = IndexSliceDisplay(shape_indices);
        f.write_fmt(format_args!(
            "#{sdr_idx} = SHAPE_DEFINITION_REPRESENTATION(#{pds_idx}, #{sr_idx});
#{pds_idx} = PRODUCT_DEFINITION_SHAPE('', '', #{pd_idx});
#{pd_idx} = PRODUCT_DEFINITION('design', '', #{pdf_idx}, #{pdc_idx});
#{pdf_idx} = PRODUCT_DEFINITION_FORMATION('', '', #{p_idx});
#{p_idx} = PRODUCT('{id}', '{name}', '{description}', (#{pc_idx}));
#{pdc_idx} = DESIGN_CONTEXT('', #{GLOBAL_APPLICATION_CONTEXT_INDEX}, 'design');
#{pc_idx} = MECHANICAL_CONTEXT('', #{GLOBAL_APPLICATION_CONTEXT_INDEX}, 'mechanical');
#{sr_idx} = SHAPE_REPRESENTATION('', {shape_indices}, #{COMMON_REPRESENTATION_CONTEXT_INDEX});\n"
        ))?;

        for display in &displays {
            Display::fmt(display, f)?;
        }
        Ok(())
    }
}

impl<'a, Model, Models, Matrix> StepLength
    for Node<'a, NodeEntity<Models, PartAttributes>, EdgeEntity<Matrix, PartAttributes>>
where
    Model: StepLength,
    for<'b> &'b Models: IntoIterator<Item = &'b Model>,
    Matrix: Copy,
    MatrixAsAxis<Matrix>: ConstStepLength,
{
    fn step_length(&self) -> usize {
        8 + MatrixAsAxis::<Matrix>::LENGTH * self.edges().len()
            + (&self.entity().shape)
                .into_iter()
                .map(|shape| shape.step_length())
                .sum::<usize>()
    }
}

// Indices into the entity-number sequence emitted for a single assembly
// node, computed once per node and looked up per outgoing edge.
#[derive(Clone, Copy, Debug)]
struct NodeInEdge {
    product_definition_idx: usize,
    shape_representation_idx: usize,
}

impl NodeInEdge {
    fn new(node_idx: usize) -> Self {
        NodeInEdge {
            product_definition_idx: node_idx + 2,
            shape_representation_idx: node_idx + 7,
        }
    }
}

// Edge-emit helper: produces the five entities for a single assembly edge
// (context-dependent shape rep, the relationship variant, identity-defined
// transformation, product-definition shape, next-assembly-usage occurrence).
#[derive(Clone, Copy, Debug)]
struct EdgeDisplay<'a> {
    matrix_idx: usize,
    attrs: &'a PartAttributes,
    nodes: (NodeInEdge, NodeInEdge),
}

impl StepFormat for EdgeDisplay<'_> {
    fn fmt(&self, idx: usize, f: &mut Formatter<'_>) -> Result {
        let cdsr_idx = idx;
        let rr_idx = cdsr_idx + 1;
        let idt_idx = rr_idx + 1;
        let mat_idx = self.matrix_idx;
        let pds_idx = idt_idx + 1;
        let nauo_idx = pds_idx + 1;

        let (
            NodeInEdge {
                product_definition_idx: pd_idx0,
                shape_representation_idx: sr_idx0,
            },
            NodeInEdge {
                product_definition_idx: pd_idx1,
                shape_representation_idx: sr_idx1,
            },
        ) = self.nodes;

        let PartAttributes {
            name,
            id,
            description,
            // An edge never carries one (it is a usage occurrence, not a
            // product), and the emitter would have nothing to do with it if it
            // did. Same reasoning as the node impl above.
            shape_representation: _,
        } = &self.attrs;

        f.write_fmt(format_args!(
            "#{cdsr_idx} = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#{rr_idx}, #{pds_idx});
#{rr_idx} = (
    REPRESENTATION_RELATIONSHIP('', '', #{sr_idx0}, #{sr_idx1})
    REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#{idt_idx})
    SHAPE_REPRESENTATION_RELATIONSHIP()
);
#{idt_idx} = ITEM_DEFINED_TRANSFORMATION('', '', #{GLOBAL_IDENTITY_MATRIX}, #{mat_idx});
#{pds_idx} = PRODUCT_DEFINITION_SHAPE('', '', #{nauo_idx});
#{nauo_idx} = NEXT_ASSEMBLY_USAGE_OCCURRENCE('{id}', '{name}', '{description}', #{pd_idx0}, #{pd_idx1}, $);\n"
        ))
    }
}

impl ConstStepLength for EdgeDisplay<'_> {
    const LENGTH: usize = 5;
}

impl StepLength for EdgeDisplay<'_> {
    #[inline]
    fn step_length(&self) -> usize { Self::LENGTH }
}

/// Complete STEP product representation derived from an [`Assembly`].
///
/// Construct with [`StepDesign::new`] from an explicit `Assembly`, or with
/// [`StepDesign::from_model`] from a single shape (creates a single-node
/// assembly). The default application context string is
/// `"generated shape data"`; override with [`StepDesign::with_application_context`].
///
/// `Display`-ing a [`StepDesign`] writes the body of the STEP `DATA;`
/// section -- entity references resolved -- without the surrounding
/// `ISO-10303-21;`/`HEADER;`/`ENDSEC;` boilerplate, which is provided by
/// [`CompleteStepDisplay`].
#[derive(Clone, Debug)]
pub struct StepDesign<Model, Models, Matrix = Matrix4> {
    assembly: Assembly<Models, PartAttributes, Matrix, PartAttributes>,
    application_context: String,
    measurement_context: StepMeasurementContext,
    _model_ty: std::marker::PhantomData<Model>,
}

impl<Model, Models, Matrix> StepDesign<Model, Models, Matrix> {
    /// Builds a `StepDesign` from an explicit assembly.
    #[inline]
    pub fn new(assembly: Assembly<Models, PartAttributes, Matrix, PartAttributes>) -> Self {
        Self {
            assembly,
            application_context: "generated shape data".to_string(),
            measurement_context: StepMeasurementContext::default(),
            _model_ty: std::marker::PhantomData,
        }
    }

    /// Builds a `StepDesign` with a caller-provided application-context string.
    #[inline]
    pub fn with_application_context(
        assembly: Assembly<Models, PartAttributes, Matrix, PartAttributes>,
        application_context: String,
    ) -> Self {
        Self {
            assembly,
            application_context,
            measurement_context: StepMeasurementContext::default(),
            _model_ty: std::marker::PhantomData,
        }
    }

    /// Overrides the length unit and distance accuracy written into the
    /// representation-context preamble. The default preserves millimetre
    /// lengths and a `1.0E-6` `distance_accuracy_value`.
    #[inline]
    pub fn with_measurement_context(mut self, context: StepMeasurementContext) -> Self {
        self.measurement_context = context;
        self
    }

    /// Returns the length unit and distance accuracy written into the preamble.
    #[inline]
    pub fn measurement_context(&self) -> StepMeasurementContext { self.measurement_context }
}

impl<Model> StepDesign<Model, Option<Model>, Matrix4> {
    /// Wraps a single model in a one-node assembly.
    pub fn from_model(model: Model) -> Self {
        let mut assembly = Assembly::new();
        assembly.create_node(NodeEntity {
            shape: Some(model),
            attrs: PartAttributes::default(),
        });
        Self::new(assembly)
    }
}

impl<Model, Models, Matrix> Display for StepDesign<Model, Models, Matrix>
where
    Model: StepFormat + StepLength,
    for<'a> &'a Models: IntoIterator<Item = &'a Model>,
    Matrix: Copy + monstertruck_modeling::base::One,
    MatrixAsAxis<Matrix>: StepFormat + ConstStepLength,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        use std::collections::HashMap;
        let representation_context_display = StepDisplay::new(
            MonstertruckRepresentationContext(self.measurement_context),
            COMMON_REPRESENTATION_CONTEXT_INDEX,
        );
        let application_context = &self.application_context;
        let identity_display =
            StepDisplay::new(MatrixAsAxis(Matrix::one()), GLOBAL_IDENTITY_MATRIX);
        f.write_fmt(format_args!(
            "#{GLOBAL_APPLICATION_CONTEXT_INDEX} = APPLICATION_CONTEXT('{application_context}');
{representation_context_display}{identity_display}",
        ))?;

        let mut idx = GLOBAL_IDENTITY_MATRIX + MatrixAsAxis::<Matrix>::LENGTH;
        let mut node_map = HashMap::new();
        let mut matrix_map = HashMap::new();
        for node in self.assembly.all_nodes() {
            node_map.insert(node.index(), NodeInEdge::new(idx));

            let mut cursor = idx + 8;
            for (edge_idx, _) in node.edges().enumerate() {
                matrix_map.insert((node.index(), edge_idx), cursor);
                cursor += MatrixAsAxis::<Matrix>::LENGTH;
            }

            StepFormat::fmt(&node, idx, f)?;
            idx += node.step_length();
        }

        for node in self.assembly.all_nodes() {
            for (i, edge) in node.edges().enumerate() {
                let (node_idx0, node_idx1) = edge.nodes();
                // SAFETY: `node_map` was filled in the loop above for every
                // node returned by `all_nodes`, and the edge endpoints
                // came from that same iterator.
                let nodes = (
                    *node_map.get(&node_idx0).unwrap(),
                    *node_map.get(&node_idx1).unwrap(),
                );

                let matrix_key = (node.index(), i);
                // SAFETY: `matrix_map` was filled in the loop above for every
                // (node, edge_index) pair we visit here.
                let matrix_idx = *matrix_map.get(&matrix_key).unwrap();

                let edge_display = EdgeDisplay {
                    nodes,
                    matrix_idx,
                    attrs: edge.attributes(),
                };
                StepFormat::fmt(&edge_display, idx, f)?;
                idx += EdgeDisplay::LENGTH;
            }
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
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
}
