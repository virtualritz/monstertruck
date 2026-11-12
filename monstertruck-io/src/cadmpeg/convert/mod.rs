//! The converter: [`cadmpeg_ir::CadIr`] to monstertruck bodies.
//!
//! # Nothing is dropped, and nothing usable is thrown away
//!
//! Handing back three bodies for a file that holds five is the worst available
//! outcome: the caller sees success, gets geometry, and has no way to learn what
//! is missing. So no refusal here is ever silent.
//!
//! The first version of this module drew the wrong conclusion from that and
//! failed the WHOLE call on any single unconvertible body. Safe, and useless: a 28
//! MB IGES 5.2 export measured 665 sheet bodies and 1 construction wire, and the
//! strict rule returned an error for a file that is 665/666 convertible. Real
//! IGES carries wires beside its surfaces as a matter of course.
//!
//! So [`import`] keeps what converts and NAMES what does not, in
//! [`Import::refused`]. Both properties hold at once: a caller cannot be misled
//! about completeness, and is not denied the geometry that was there.
//!
//! The split is by scope, not by severity:
//!
//! * a **body**-level refusal is collected -- it says nothing about the others;
//! * a **document**-level failure propagates. A dangling reference means the
//!   decoded graph is not closed, and there is no honest partial answer to give
//!   from an open graph.
//!
//! [`convert`] is the strict reading, kept for callers who genuinely want
//! all-or-nothing, and it is now built ON TOP of [`import`] rather than beside it,
//! so the two cannot disagree about what is convertible.
//!
//! The refusals in [`surface`], [`curve`] and [`topology`] are typed errors rather
//! than `None` for the same reason: a reason that reaches the caller is worth more
//! than an absence.
//!
//! The claim is about STRUCTURAL entities: bodies, faces, edges and vertices
//! either convert or are named. The compressed representation itself has nowhere
//! to carry pcurves, per-entity tolerances, names, colours or visibility, so
//! those are representation limits rather than decoder losses and are not
//! reported. Pcurves are re-derived downstream; the rest are outside what a
//! [`CompressedShell`] models.
//!
//! [`CompressedShell`]: monstertruck_topology::compress::CompressedShell
//!
//! # Every body appears wherever the assembly places it
//!
//! Since cadmpeg 0.6 an assembly is a product structure: reusable
//! definitions own bodies, and a tree of occurrences places each definition,
//! once per use. A body therefore exists ONCE in `model.bodies` however many
//! times the assembly uses it. Reading only `model.bodies` put an instanced part
//! at one place -- its definition's own coordinates -- and silently dropped
//! every other use; worse, it put even a part used ONCE at its definition's
//! coordinates rather than where its occurrence places it.
//!
//! So [`import`] expands the product structure: a body owned by a definition is
//! emitted once per occurrence of that definition, placed by the occurrence's
//! world transform (its chain of placements from the root) composed with the
//! body's own placement. A body that no definition owns, or whose definitions
//! are never placed, is emitted once at its own placement, as before.

mod frame;

pub(super) mod curve;
pub(super) mod surface;
pub(super) mod topology;

#[cfg(test)]
mod tests;

use std::collections::HashMap;

use cadmpeg_ir::geometry::{Curve as IrCurve, Surface as IrSurface};
use cadmpeg_ir::index::{ModelIndex, StandardIndex};
use cadmpeg_ir::products::{AssemblyGraph, Occurrence, OccurrenceParent, PrototypeReference};
use cadmpeg_ir::topology::{
    Body, BodyKind, Coedge, Edge as IrEdge, Face as IrFace, Loop as IrLoop, Point as IrPoint,
    Vertex as IrVertex,
};
use cadmpeg_ir::transform::Transform as IrTransform;

use super::{Import, ImportedBody, ImportedSheet, ImportedSolid, Refusal};
use crate::{Error, Result};

/// What a conversion needs to know throughout: the indexed document and which
/// format it came from.
///
/// The format name is carried so every refusal says `IGES carried a ...` rather
/// than making the caller guess which of two readers produced it.
pub(super) struct Context<'a> {
    index: ModelIndex<'a>,
    format: &'static str,
}

impl<'a> Context<'a> {
    /// Index the document's MODEL arenas only. The native namespaces are never
    /// looked up by the converter, so indexing them would be work for nothing.
    fn new(ir: &'a cadmpeg_ir::CadIr, format: &'static str) -> Self {
        Self {
            index: ModelIndex::new_model_only(ir),
            format,
        }
    }

    fn malformed(&self, detail: String) -> Error {
        Error::MalformedGeometry {
            format: self.format,
            detail,
        }
    }

    fn dangling(&self, kind: &'static str, id: &str) -> Error {
        Error::DanglingReference {
            format: self.format,
            kind,
            id: id.to_owned(),
        }
    }
}

/// Resolve one reference, or report which table it dangled out of.
///
/// A macro rather than a generic function because `ModelIndex`'s accessors are
/// one per table, generated from the same declaration as the tables themselves.
/// `StandardIndex` is the query policy with no work budget, whose lookups are
/// infallible -- the decode that produced the document already enforced its
/// budget.
macro_rules! resolvers {
    ($($method:ident -> $table:ident: $entity:ty, $kind:literal;)*) => {
        impl<'a> Context<'a> {
            $(
                fn $method(&self, id: &str) -> Result<&'a $entity> {
                    self.index
                        .$table(id, StandardIndex)
                        .ok_or_else(|| self.dangling($kind, id))
                }
            )*
        }
    };
}

resolvers! {
    face -> faces: IrFace, "face";
    boundary_loop -> loops: IrLoop, "loop";
    coedge -> coedges: Coedge, "coedge";
    edge -> edges: IrEdge, "edge";
    vertex -> vertices: IrVertex, "vertex";
    point -> points: IrPoint, "point";
    surface -> surfaces: IrSurface, "surface";
    curve -> curves: IrCurve, "curve";
    region -> regions: cadmpeg_ir::topology::Region, "region";
    shell -> shells: cadmpeg_ir::topology::Shell, "shell";
    procedural_surface -> procedural_surfaces: cadmpeg_ir::geometry::ProceduralSurface, "procedural surface";
    procedural_curve -> procedural_curves: cadmpeg_ir::geometry::ProceduralCurve, "procedural curve";
}

/// Convert every body in a decoded document, keeping what converts.
///
/// A body-level refusal is collected, not propagated. A DOCUMENT-level failure
/// still propagates: a dangling reference means the decoded graph is not closed,
/// and an occurrence tree with a cycle or a missing parent places nothing
/// honestly, so neither has a partial answer worth returning.
///
/// The returned [`Import`] carries no decoder losses: this function sees only
/// the document. The decode entry points attach the decoder's report.
pub(super) fn import(ir: &cadmpeg_ir::CadIr, format: &'static str) -> Result<Import> {
    let context = Context::new(ir, format);
    let placements = Placements::of(ir, &context)?;
    let mut bodies = Vec::new();
    let mut refused = Vec::new();
    for body in &ir.model.bodies {
        let instances = placements.instances_of(body, &context);
        let converted = match body_to_imported(body, &context) {
            Ok(converted) => converted,
            // A dangling reference is about the DOCUMENT, not this body, so it
            // fails the call. Anything else is this body's own problem.
            Err(error @ Error::DanglingReference { .. }) => return Err(error),
            Err(reason) => {
                let count = instances.len().max(1);
                refused.push(Refusal {
                    body: match count {
                        1 => body.id.to_string(),
                        count => format!("{} (all {count} instances)", body.id),
                    },
                    kind: body_kind_name(body.kind),
                    instances: count,
                    reason,
                });
                continue;
            }
        };
        for instance in instances {
            let label = || match &instance.occurrence {
                Some(occurrence) => format!("{} at occurrence {occurrence}", body.id),
                None => body.id.to_string(),
            };
            match instance.transform {
                Ok(None) => bodies.push(converted.clone()),
                Ok(Some(transform)) => {
                    let mut placed = converted.clone();
                    match place(&mut placed, &transform, &context) {
                        Ok(()) => bodies.push(placed),
                        Err(reason) => refused.push(Refusal {
                            body: label(),
                            kind: body_kind_name(body.kind),
                            instances: 1,
                            reason,
                        }),
                    }
                }
                Err(reason) => refused.push(Refusal {
                    body: label(),
                    kind: body_kind_name(body.kind),
                    instances: 1,
                    reason,
                }),
            }
        }
    }
    Ok(Import {
        bodies,
        refused,
        losses: Vec::new(),
    })
}

/// One place a body is to appear, and the placement that puts it there.
struct Instance {
    /// The occurrence that places it, when the product structure does.
    occurrence: Option<String>,
    /// The world placement to apply, `None` for the identity, or why this
    /// instance cannot be placed.
    transform: Result<Option<IrTransform>>,
}

/// The product structure, read once: which definitions own each body, and the
/// world placement of every occurrence of every definition.
struct Placements<'a> {
    /// Body id to the ids of the definitions that own it.
    owners: HashMap<&'a str, Vec<&'a str>>,
    /// Definition id to the occurrences that place it, in source order.
    uses: HashMap<&'a str, Vec<&'a Occurrence>>,
    /// Occurrence id to its world placement, or why it has none.
    world: HashMap<&'a str, std::result::Result<IrTransform, String>>,
}

impl<'a> Placements<'a> {
    fn of(ir: &'a cadmpeg_ir::CadIr, context: &Context<'_>) -> Result<Self> {
        let occurrences = &ir.model.occurrences;
        // Validates the tree: unique ids, every parent present, no cycle, and
        // every composed placement finite. Any failure is about the DOCUMENT.
        AssemblyGraph::new(occurrences).map_err(|error| {
            context.malformed(format!(
                "the assembly's occurrence tree is invalid: {error}"
            ))
        })?;
        let by_id: HashMap<&'a str, &'a Occurrence> = occurrences
            .iter()
            .map(|occurrence| (occurrence.id.as_str(), occurrence))
            .collect();
        let mut world = HashMap::new();
        for occurrence in occurrences {
            // Memoised into `world`, failures included; the return value is
            // read back per instance.
            let _resolved = world_of(occurrence, &by_id, &mut world);
        }
        let mut owners: HashMap<&str, Vec<&str>> = HashMap::new();
        for definition in &ir.model.product_definitions {
            for body in &definition.bodies {
                owners
                    .entry(body.as_str())
                    .or_default()
                    .push(definition.id.as_str());
            }
        }
        let mut uses: HashMap<&str, Vec<&Occurrence>> = HashMap::new();
        for occurrence in occurrences {
            // An external or unresolved prototype names nothing in this
            // document, so it can place none of its bodies.
            if let PrototypeReference::Local { definition } = &occurrence.prototype {
                uses.entry(definition.as_str())
                    .or_default()
                    .push(occurrence);
            }
        }
        Ok(Self {
            owners,
            uses,
            world,
        })
    }

    /// Every place `body` appears.
    ///
    /// Never empty: a body that no definition owns, or whose owning definitions
    /// are never placed, appears once, at its own placement.
    fn instances_of(&self, body: &Body, context: &Context<'_>) -> Vec<Instance> {
        let local = body.transform;
        let placed: Vec<&Occurrence> = self
            .owners
            .get(body.id.as_str())
            .into_iter()
            .flatten()
            .filter_map(|definition| self.uses.get(definition))
            .flatten()
            .copied()
            .collect();
        if placed.is_empty() {
            return vec![Instance {
                occurrence: None,
                transform: Ok(local),
            }];
        }
        placed
            .into_iter()
            .map(|occurrence| Instance {
                occurrence: Some(occurrence.id.to_string()),
                transform: self.world_placement(occurrence, local, context),
            })
            .collect()
    }

    /// The occurrence's world placement composed with the body's own.
    fn world_placement(
        &self,
        occurrence: &Occurrence,
        local: Option<IrTransform>,
        context: &Context<'_>,
    ) -> Result<Option<IrTransform>> {
        let world = match self.world.get(occurrence.id.as_str()) {
            Some(Ok(world)) => *world,
            Some(Err(reason)) => {
                return Err(Error::UnplaceableOccurrence {
                    format: context.format,
                    kind: unplaceable_kind(reason),
                });
            }
            None => {
                return Err(context.malformed(format!(
                    "occurrence {} has no resolved placement",
                    occurrence.id
                )));
            }
        };
        let composed = match local {
            Some(local) => world.compose(local).map_err(|error| {
                context.malformed(format!(
                    "occurrence {} cannot place its body: {error}",
                    occurrence.id
                ))
            })?,
            None => world,
        };
        Ok(Some(composed))
    }
}

/// The reason an occurrence chain cannot be placed, as a stable refusal word.
fn unplaceable_kind(reason: &str) -> &'static str {
    if reason == SCALED {
        "scaled assembly occurrence"
    } else {
        "unplaceable assembly occurrence"
    }
}

const SCALED: &str = "scaled";

/// Resolve one occurrence's world placement, memoised. The tree was validated
/// acyclic with every parent present, so the recursion terminates.
///
/// An occurrence with a per-axis `scale` other than one is NOT placed: the
/// representation does not say whether the scale applies before or after the
/// occurrence's transform, and AssemblyGraph's own composition ignores it. A
/// guess would put the part at the wrong size, so it is refused by name. So is
/// every occurrence below it, whose placement composes through it.
fn world_of<'a>(
    occurrence: &'a Occurrence,
    by_id: &HashMap<&'a str, &'a Occurrence>,
    world: &mut HashMap<&'a str, std::result::Result<IrTransform, String>>,
) -> std::result::Result<IrTransform, String> {
    if let Some(known) = world.get(occurrence.id.as_str()) {
        return known.clone();
    }
    let resolved = (|| {
        if occurrence.scale.iter().any(|axis| axis.get() != 1.0) {
            return Err(SCALED.to_owned());
        }
        let parent = match &occurrence.parent {
            OccurrenceParent::Root {} => IrTransform::identity(),
            OccurrenceParent::Occurrence { occurrence: parent } => {
                let parent: &'a Occurrence = by_id
                    .get(parent.as_str())
                    .copied()
                    .ok_or_else(|| format!("missing parent {parent}"))?;
                world_of(parent, by_id, world)?
            }
        };
        occurrence
            .effective_transform()
            .and_then(|local| parent.compose(local))
            .map_err(|error| error.to_string())
    })();
    world.insert(occurrence.id.as_str(), resolved.clone());
    resolved
}

/// The body kind as a stable, greppable word for a refusal summary.
fn body_kind_name(kind: BodyKind) -> &'static str {
    match kind {
        BodyKind::Solid => "solid",
        BodyKind::Sheet => "sheet",
        BodyKind::Wire => "wire",
        BodyKind::General => "mixed-dimensional",
    }
}

/// Convert every body, failing on the first that cannot be represented.
pub(super) fn convert(ir: &cadmpeg_ir::CadIr, format: &'static str) -> Result<Vec<ImportedBody>> {
    let recovered = import(ir, format)?;
    if let Some(first) = recovered.refused.into_iter().next() {
        return Err(first.reason);
    }
    Ok(recovered.bodies)
}

/// Convert one body in its DEFINITION's coordinates. Placement is applied per
/// instance by the caller.
fn body_to_imported(body: &Body, context: &Context<'_>) -> Result<ImportedBody> {
    match body.kind {
        BodyKind::Solid => Ok(ImportedBody::Solid(solid(body, context)?)),
        BodyKind::Sheet => Ok(ImportedBody::Sheet(sheet(body, context)?)),
        // A wire body is a curve collection with no faces, and a `General` body
        // mixes dimensions. Neither has a compressed form. Named, not dropped.
        BodyKind::Wire => Err(Error::UnsupportedBodyKind {
            format: context.format,
            kind: "wire",
        }),
        BodyKind::General => Err(Error::UnsupportedBodyKind {
            format: context.format,
            kind: "mixed-dimensional",
        }),
        // Exhaustive on purpose: see the note in [`surface::convert`].
    }
}

/// A solid: one compressed shell per source shell, because for a body that bounds
/// a volume the shell partition IS the difference between the outer surface and a
/// void.
fn solid(body: &Body, context: &Context<'_>) -> Result<ImportedSolid> {
    let mut boundaries = Vec::new();
    for region in &body.regions {
        let region = context.region(region.as_str())?;
        for shell in &region.shells {
            let shell = context.shell(shell.as_str())?;
            let mut builder = topology::ShellBuilder::new(context);
            builder.add_shell(shell)?;
            if builder.is_empty() {
                return Err(context.malformed(format!(
                    "solid body {} has a shell with no faces, so it bounds no volume",
                    body.id
                )));
            }
            boundaries.push(builder.finish());
        }
    }
    if boundaries.is_empty() {
        return Err(context.malformed(format!("solid body {} carries no boundary shell", body.id)));
    }
    Ok(ImportedSolid {
        boundaries,
        id_allocator: None,
        attributes: None,
    })
}

/// A sheet: every shell of the body merged into one compressed shell.
///
/// See [`topology::ShellBuilder::add_shell`] for why merging is right here and
/// wrong for a solid.
fn sheet(body: &Body, context: &Context<'_>) -> Result<ImportedSheet> {
    let mut builder = topology::ShellBuilder::new(context);
    for region in &body.regions {
        let region = context.region(region.as_str())?;
        for shell in &region.shells {
            let shell = context.shell(shell.as_str())?;
            builder.add_shell(shell)?;
        }
    }
    if builder.is_empty() {
        return Err(context.malformed(format!("sheet body {} carries no face", body.id)));
    }
    Ok(builder.finish())
}

/// Apply a world placement to a converted body.
///
/// A placement applies to every vertex, curve and surface in the body. Leaving
/// it out puts a correctly-shaped body at the origin instead of where the
/// assembly says it is -- which is a silent wrong answer, not a visible failure.
fn place(body: &mut ImportedBody, transform: &IrTransform, context: &Context<'_>) -> Result<()> {
    let matrix = frame::matrix(transform);
    match body {
        ImportedBody::Solid(solid) => solid
            .boundaries
            .iter_mut()
            .try_for_each(|shell| place_shell(shell, matrix, context)),
        ImportedBody::Sheet(sheet) => place_shell(sheet, matrix, context),
    }
}

fn place_shell(
    shell: &mut ImportedSheet,
    matrix: monstertruck_modeling::Matrix4,
    context: &Context<'_>,
) -> Result<()> {
    use monstertruck_modeling::{SquareMatrix, Transform as _, Transformed};
    // A placement maps the surface's partials by the linear part, and
    // monstertruck's `Processor::normal` divides by the determinant, so a
    // NEGATIVE determinant flips the transformed surface's natural normal
    // relative to the oriented normal the source stated. The face flag must
    // absorb that sign or a mirrored body becomes an inside-out solid with
    // every topological check still green.
    let determinant = matrix.determinant();
    if !determinant.is_finite() || determinant == 0.0 {
        return Err(Error::UnplaceableOccurrence {
            format: context.format,
            kind: "singular placement",
        });
    }
    let mirrored = determinant < 0.0;
    for vertex in &mut shell.vertices {
        *vertex = matrix.transform_point(*vertex);
    }
    for edge in &mut shell.edges {
        edge.curve.transform_by(matrix);
    }
    for face in &mut shell.faces {
        face.surface.transform_by(matrix);
        if mirrored {
            face.orientation = !face.orientation;
        }
    }
    Ok(())
}
