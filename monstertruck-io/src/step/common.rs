//! Types shared between the [`load`](crate::step::load) and [`save`](crate::step::save) sides of STEP I/O.

/// Identifying attributes attached to a STEP part (product or component).
///
/// Surfaced by the load side as the `attrs` payload of a STEP assembly node
/// or edge, and consumed by the save side when emitting `PRODUCT*` /
/// `SHAPE_DEFINITION_REPRESENTATION` entities.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct PartAttributes {
    /// `id` of the part. Mirrors the STEP `PRODUCT.id` field.
    pub id: String,
    /// Human-readable name of the part.
    pub name: String,
    /// Free-form description of the part.
    pub description: String,
    /// STEP entity id of the `SHAPE_REPRESENTATION` this part's shape was read
    /// out of -- **a load-side identity, an index into the file it came from,
    /// and never something the save side writes.**
    ///
    /// # Why it is here
    ///
    /// [`Table::step_assy`](crate::step::load::Table::step_assy) resolves
    /// `shape_definition_representation.used_representation` to get a node's
    /// `shape`, and until spec 012 it threw the representation's own id away.
    /// A caller holding an assembly node then had to REVERSE-MATCH the whole
    /// item list back to a representation
    /// ([`Table::shape_representation_of_items`](crate::step::load::Table::shape_representation_of_items))
    /// before it could take the part -> solid hop
    /// ([`Table::solids_via_shape_relationship`](crate::step::load::Table::solids_via_shape_relationship)).
    /// That works, but it is a search where a field will do, and a search can
    /// be ambiguous where the id cannot.
    ///
    /// # Why it does not round-trip
    ///
    /// The save side allocates its OWN entity numbering, so a
    /// `SHAPE_REPRESENTATION` id read out of one file names nothing in the file
    /// the save side emits. Writing it back would be a dangling reference. Both
    /// `StepFormat` impls in [`crate::step::save`] therefore destructure this field to
    /// `_` on purpose: **load -> save -> load is byte-identical whether this
    /// field is populated or `None`**, and the id a re-load carries is the one
    /// belonging to the NEW file. Pinned by
    /// `the_representation_id_never_reaches_the_save_side`.
    ///
    /// # Who legitimately has none
    ///
    /// - **Assembly EDGES.** [`crate::step::load::convert::AssembleEntity`] reuses this
    ///   type for `NEXT_ASSEMBLY_USAGE_OCCURRENCE` attributes, and an edge is a
    ///   usage occurrence, not a product with a shape representation.
    /// - **Anything the save side or an application builds from scratch**,
    ///   including [`Default`] and `StepDesign::from_model`: there is no STEP
    ///   file to be an id into.
    ///
    /// Every node [`Table::step_assy`](crate::step::load::Table::step_assy) returns
    /// carries `Some`, and the id is always one the table holds -- the loader
    /// resolves the record before it records the id, and errors out otherwise.
    pub shape_representation: Option<u64>,
}

// `PartAttrs` is the upstream `truck-stepio` spelling. Abbreviating
// `Attributes` to `Attrs` saves three characters of typing at the cost of
// reading clarity at every call site; we standardise on the non-abbreviated
// name in public APIs and keep the alias only so existing callers (and code
// ported from `truck-stepio`) continue to compile. Slated for removal once
// downstream callers have moved off the old name.
#[deprecated(since = "0.3.1", note = "renamed to `PartAttributes`.")]
pub use PartAttributes as PartAttrs;
