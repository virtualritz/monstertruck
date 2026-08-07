//! Product structure and assembly entities.
//!
//! Contexts, products, product definitions, shape representations and the
//! transformations that relate them -- the non-geometric scaffolding that
//! says which shape belongs to which part, and where.

use super::*;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = application_context)]
#[holder(generate_deserialize)]
pub struct ApplicationContext {
    pub application: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = product_context)]
#[holder(generate_deserialize)]
pub struct ProductContext {
    pub name: String,
    #[holder(use_place_holder)]
    pub frame_of_reference: ApplicationContext,
    pub discipline_type: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = product)]
#[holder(generate_deserialize)]
pub struct Product {
    pub id: String,
    pub name: String,
    /// `OPTIONAL text` in ISO 10303-41 `product`, so `$` is CONFORMANT here.
    /// Measured: 115 of 225 `PRODUCT` records in `Scania-8x4.stp` and 27 of 180
    /// in `Scania-Engine-V8-XT-Turbo.step` write `$`; while this was `String`
    /// every one of them was refused and dropped, taking the product out of
    /// `Table::product` and the part out of the assembly graph.
    pub description: Option<String>,
    #[holder(use_place_holder)]
    pub frame_of_reference: Vec<ProductContext>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = product_definition_formation)]
#[holder(generate_deserialize)]
pub struct ProductDefinitionFormation {
    pub id: String,
    /// `OPTIONAL text` in ISO 10303-41 `product_definition_formation`, so `$` is
    /// CONFORMANT. Measured: **100%** of the 225 + 180 records in the two Scania
    /// files write `$` here, which is why `Table::product_definition_formation`
    /// was completely empty for both and `Table::step_assy` had nothing to walk.
    pub description: Option<String>,
    #[holder(use_place_holder)]
    pub of_product: Product,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = product_definition_context)]
#[holder(generate_deserialize)]
pub struct ProductDefinitionContext {
    pub name: String,
    #[holder(use_place_holder)]
    pub frame_of_reference: ApplicationContext,
    pub life_cycle_stage: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = product_definition)]
#[holder(generate_deserialize)]
pub struct ProductDefinition {
    pub id: String,
    pub description: String,
    #[holder(use_place_holder)]
    pub formation: ProductDefinitionFormation,
    #[holder(use_place_holder)]
    pub frame_of_reference: ProductDefinitionContext,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(generate_deserialize)]
pub enum CharacterizedDefinition {
    #[holder(use_place_holder)]
    ProductDefinition(Box<ProductDefinition>),
    #[holder(use_place_holder)]
    ProductDefinitionShape(Box<ProductDefinitionShape>),
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = product_definition_shape)]
#[holder(generate_deserialize)]
pub struct ProductDefinitionShape {
    /// `label`, and NOT optional in ISO 10303-41 `property_definition` -- so a
    /// `$` here is the EXPORTER being non-conformant, not the schema allowing it.
    /// Accepted anyway, and the reason is measured: 470 of 695 records in
    /// `Scania-8x4.stp` and 726 of 906 in `Scania-Engine-V8-XT-Turbo.step` are
    /// spelled `PRODUCT_DEFINITION_SHAPE($,$,#..)`. Refusing them is refusing the
    /// file's entire assembly over two display strings, which is the same trade
    /// [`Table::from_step_bytes`](crate::step::load::Table::from_step_bytes) already
    /// declines to make for its encoding fallback. `Option` rather than an empty
    /// `String` so the distinction between "unset" and "set to empty" survives --
    /// 225 records in the same file really do write `''`.
    pub name: Option<String>,
    /// `OPTIONAL text` in ISO 10303-41 `property_definition`: `$` is CONFORMANT.
    /// Measured at 100% of both files' records.
    pub description: Option<String>,
    #[holder(use_place_holder)]
    pub definition: CharacterizedDefinition,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = shape_representation)]
#[holder(generate_deserialize)]
pub struct ShapeRepresentation {
    pub name: String,
    #[holder(use_place_holder)]
    pub items: Vec<RepresentationItem>,
    #[holder(use_place_holder)]
    pub context_of_items: RepresentationContext,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = context_dependent_shape_representation)]
#[holder(generate_deserialize)]
pub struct ContextDependentShapeRepresentation {
    #[holder(use_place_holder)]
    pub representation_relation: ShapeRepresentationRelationshipWithTransformation,
    #[holder(use_place_holder)]
    pub represented_product_relation: ProductDefinitionShape,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = shape_definition_representation)]
#[holder(generate_deserialize)]
pub struct ShapeDefinitionRepresentation {
    #[holder(use_place_holder)]
    pub definition: ProductDefinitionShape,
    #[holder(use_place_holder)]
    pub used_representation: ShapeRepresentation,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = shape_representation_relationship)]
#[holder(generate_deserialize)]
pub struct ShapeRepresentationRelationship {
    /// `label`, mandatory in ISO 10303-43 `representation_relationship`; accepted
    /// as unset for the same measured reason as
    /// [`ProductDefinitionShape::name`].
    pub name: Option<String>,
    /// `OPTIONAL text` in ISO 10303-43 `representation_relationship`: `$` is
    /// CONFORMANT.
    pub description: Option<String>,
    #[holder(use_place_holder)]
    pub rep_1: ShapeRepresentation,
    #[holder(use_place_holder)]
    pub rep_2: ShapeRepresentation,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = shape_representation_relationship_with_transformation)]
#[holder(generate_deserialize)]
pub struct ShapeRepresentationRelationshipWithTransformation {
    /// `label`, mandatory in ISO 10303-43; accepted as unset. Measured: **100%**
    /// of the 470 + 726 `REPRESENTATION_RELATIONSHIP` sub-records of the complex
    /// `SHAPE_REPRESENTATION_RELATIONSHIP` instances in the two Scania files are
    /// spelled `REPRESENTATION_RELATIONSHIP($,$,#..,#..)`. These are the records
    /// that ATTACH the placement matrices to the parts, so all 1,086 solids lost
    /// their position to this one refusal.
    pub name: Option<String>,
    /// `OPTIONAL text` in ISO 10303-43: `$` is CONFORMANT.
    pub description: Option<String>,
    #[holder(use_place_holder)]
    pub rep_1: ShapeRepresentation,
    #[holder(use_place_holder)]
    pub rep_2: ShapeRepresentation,
    #[holder(use_place_holder)]
    pub transformation_operator: ItemDefinedTransformation,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = next_assembly_usage_occurrence)]
#[holder(generate_deserialize)]
pub struct NextAssemblyUsageOccurrence {
    pub id: String,
    pub name: String,
    pub description: String,
    #[holder(use_place_holder)]
    pub relating_product_definition: ProductDefinition,
    #[holder(use_place_holder)]
    pub related_product_definition: ProductDefinition,
    pub reference_designator: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, Holder)]
#[holder(table = Table)]
#[holder(field = item_defined_transformation)]
#[holder(generate_deserialize)]
pub struct ItemDefinedTransformation {
    /// `label`, mandatory in ISO 10303-43 `item_defined_transformation`; accepted
    /// as unset. Measured: **100%** of the 470 + 726 records in the two Scania
    /// files are spelled `ITEM_DEFINED_TRANSFORMATION($,$,#..,#..)`. This entity
    /// carries the assembly's placement matrices.
    name: Option<String>,
    /// `OPTIONAL text` in ISO 10303-43: `$` is CONFORMANT.
    description: Option<String>,
    #[holder(use_place_holder)]
    transform_item_1: Axis2Placement,
    #[holder(use_place_holder)]
    transform_item_2: Axis2Placement,
}

impl TryFrom<&ItemDefinedTransformation> for Matrix3 {
    type Error = StepConvertingError;
    fn try_from(value: &ItemDefinedTransformation) -> Result<Self, Self::Error> {
        let mat1: Self = (&value.transform_item_1).try_into()?;
        let mat2: Self = (&value.transform_item_2).try_into()?;
        let inv = mat1
            .invert()
            .ok_or("failed to invert transform_item_1 Matrix3")?;
        Ok(mat2 * inv)
    }
}

impl TryFrom<&ItemDefinedTransformation> for Matrix4 {
    type Error = StepConvertingError;
    fn try_from(value: &ItemDefinedTransformation) -> Result<Self, Self::Error> {
        let mat1: Self = (&value.transform_item_1).try_into()?;
        let mat2: Self = (&value.transform_item_2).try_into()?;
        let inv = mat1
            .invert()
            .ok_or("failed to invert transform_item_1 Matrix4")?;
        Ok(mat2 * inv)
    }
}
