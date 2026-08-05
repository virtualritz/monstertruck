//! Typed accounting for what a STEP load KEPT and what it LOST.
//!
//! # Why this exists
//!
//! Every loss path on the STEP load route used to end in
//! `.map_err(|e| eprintln!("{e}")).ok()?` inside a `filter_map`, or -- worse --
//! in a bare `?` on an `Option` with no message at all. The consequence, measured
//! over the 8-file corpus and the 15 in-repo fixtures in spec 011 Phase 0
//! (`specs/011-step-coverage-closure/evidence/conversion-census.md`):
//!
//! - `Table::from_step_bytes` returns `Ok` having dropped 4,562 entity records on
//!   the two Scania files -- 100% of their assembly graph.
//! - `Table::to_compressed_shell` returns `Ok` with faces missing (253 corpus
//!   faces) and with boundary wires missing (10 of 160 on
//!   `boxy-with-surfacetex.step`, an IN-GATE fixture).
//!
//! In both cases the only evidence was on stderr, and in the `VERTEX_LOOP` case
//! there was not even that. A caller could not ask "did I get everything?" and
//! get a truthful answer. These types are that answer.
//!
//! # Shape of the API
//!
//! Purely ADDITIVE, and loss is never an error by default:
//!
//! - [`Table::entity_report`](crate::step::load::Table::entity_report) -- what the
//!   table-building pass swallowed, tallied per STEP entity type.
//! - `Table::to_compressed_shell_reported` and its siblings -- the converted
//!   value PLUS a [`ShellLoadReport`]. The un-suffixed methods still exist,
//!   still return exactly what they returned before, and simply discard the
//!   report.
//! - [`ShellLoadReport::require_lossless`] / [`EntityLoadReport::require_empty`]
//!   turn loss into a typed [`LoadError`](crate::step::load::LoadError) for the callers
//!   that want strictness. Opt-in, never the default: every in-gate fixture is
//!   lossy today (see the `VERTEX_LOOP` note on
//!   [`LossReason::DegenerateVertexLoop`]), so a default-strict load would refuse
//!   files the kernel handles correctly.
//!
//! Both reports are additive ([`ShellLoadReport::merge`]) so a caller can roll a
//! solid's shells up into one figure, and both `Display` as a one-line summary.

use std::collections::BTreeMap;
use std::fmt;

/// WHAT was lost. One variant per level of the compressed-shell structure, so a
/// caller can tell "I lost a whole face" from "I lost one boundary of a face I
/// kept" -- the two have very different consequences downstream.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum LossCategory {
    /// A `FACE_SURFACE`/`ADVANCED_FACE` listed by the shell. Losing one leaves a
    /// hole: the `CompressedShell` is no longer watertight.
    Face,
    /// A boundary loop (`FACE_BOUND`/`FACE_OUTER_BOUND`) of a face that WAS kept.
    /// Losing one widens the face's trimmed region.
    Wire,
    /// One edge-use inside a wire that WAS kept. Losing one leaves an open wire.
    EdgeUse,
    /// A distinct `EDGE_CURVE` of the shell. Indices into
    /// `CompressedShell::edges` are assigned only to edges that were kept.
    Edge,
    /// A distinct `VERTEX_POINT` of the shell. Indices into
    /// `CompressedShell::vertices` are assigned only to vertices that were kept.
    Vertex,
    /// A representation the part -> solid hop should have opened -- the `rep_2`
    /// of a non-transforming `SHAPE_REPRESENTATION_RELATIONSHIP`. Losing one
    /// means the hop could not even LOOK for that part's solids. See
    /// [`Table::solids_via_shape_relationship`](crate::step::load::Table::solids_via_shape_relationship).
    Representation,
    /// An item listed by a representation the hop DID open. `kept` counts the
    /// ones that are solids; anything else is reported lost with
    /// [`LossReason::RepresentationItemNotASolid`], which is frequently benign
    /// -- see that variant.
    Solid,
}

impl LossCategory {
    /// Presentation order, and the order [`ShellLoadReport`] iterates in.
    pub const ALL: [Self; 7] = [
        Self::Face,
        Self::Wire,
        Self::EdgeUse,
        Self::Edge,
        Self::Vertex,
        Self::Representation,
        Self::Solid,
    ];

    /// The categories a SHELL conversion can touch -- [`Self::ALL`] minus the
    /// two the part -> solid hop owns.
    ///
    /// Split out so a caller can still say "the conversion walked every level it
    /// has" without that claim silently weakening when a category is added for a
    /// route that conversion never takes.
    pub const SHELL: [Self; 5] = [
        Self::Face,
        Self::Wire,
        Self::EdgeUse,
        Self::Edge,
        Self::Vertex,
    ];

    /// Lower-case singular noun, for messages.
    pub fn noun(self) -> &'static str {
        match self {
            Self::Face => "face",
            Self::Wire => "wire",
            Self::EdgeUse => "edge use",
            Self::Edge => "edge",
            Self::Vertex => "vertex",
            Self::Representation => "representation",
            Self::Solid => "solid",
        }
    }

    /// Lower-case plural noun. Spelled out rather than `noun() + "s"` because
    /// "vertexs" in a diagnostic reads as a bug in the diagnostic.
    pub fn plural(self) -> &'static str {
        match self {
            Self::Face => "faces",
            Self::Wire => "wires",
            Self::EdgeUse => "edge uses",
            Self::Edge => "edges",
            Self::Vertex => "vertices",
            Self::Representation => "representations",
            Self::Solid => "solids",
        }
    }
}

/// WHY it was lost. Deliberately finer-grained than the code paths that raise
/// them, because the interesting distinction is not "which `?` fired" but
/// "is this a defect, a refusal, or a legitimate degeneracy".
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum LossReason {
    /// A shell listed a face id that resolves to no `face_surface`/`oriented_face`
    /// record, or to a non-reference parameter.
    FaceUnresolved,
    /// The face's `face_geometry` could not be resolved out of the table -- the
    /// surface record is absent, or is a subtype no `SurfaceAny` variant covers.
    SurfaceUnresolved,
    /// The surface record resolved, and the conversion to the kernel's `Surface`
    /// refused typed. This is the correct-or-typed arm working as intended at the
    /// record; the loss is that the FACE then disappears.
    SurfaceRefused,
    /// A face listed a bound id that is in no table map at all.
    BoundUnresolved,
    /// The bound is a `VERTEX_LOOP`: a single-vertex degenerate loop.
    ///
    /// **Measured (spec 011 T7), every in-repo instance:** all 19 are point
    /// boundaries at a parameterisation singularity -- 10 cone apexes
    /// (`boxy-with-surfacetex.step`), 8 sphere poles (`abc-0000.step` x7,
    /// `occt-sphere.step`), and 1 torus seam point (`occt-torus.step`). Two of
    /// them (`occt-sphere`, `occt-torus`) are the face's ONLY bound, i.e. the
    /// face is the whole closed surface.
    ///
    /// Dropping such a loop is the CORRECT outcome for a compressed shell, whose
    /// wires are sequences of edge uses: a point boundary has no edges, so
    /// representing it would mean emitting an empty wire -- an assertion that the
    /// face is bounded by something of zero extent, which is worse than saying
    /// nothing. The defect this reason fixes was the SILENCE, not the drop.
    DegenerateVertexLoop,
    /// The bound resolved to neither an `EDGE_LOOP` nor a `VERTEX_LOOP` -- a
    /// `POLY_LOOP` or another `loop` subtype the loader does not implement.
    BoundNotALoopWeImplement,
    /// A wire listed an edge id that resolves to no `edge_curve`/`oriented_edge`,
    /// or whose edge was itself lost, so no index exists for it.
    EdgeUseUnresolved,
    /// An `EDGE_CURVE`'s children could not be resolved out of the table.
    EdgeUnresolved,
    /// An `EDGE_CURVE` resolved but its geometry refused conversion to `Curve3D`.
    EdgeCurveRefused,
    /// An `EDGE_CURVE`'s start or end vertex is not a reference, or its vertex was
    /// itself lost.
    EdgeEndpointUnresolved,
    /// A `VERTEX_POINT`'s children could not be resolved out of the table.
    VertexUnresolved,
    /// A non-transforming `SHAPE_REPRESENTATION_RELATIONSHIP`'s `rep_2` is not
    /// an entity reference, or resolves to no representation the table holds.
    ///
    /// **Measured (spec 011 open item 1), every corpus instance:** the 2
    /// occurrences on `Scania-8x4.stp` and the 1 on the in-gate fixture
    /// `coffy.step` are all `GEOMETRICALLY_BOUNDED_WIREFRAME_SHAPE_REPRESENTATION`
    /// -- a subtype the loader has no arm for, so the record sits in
    /// [`Table::dummy`](crate::step::load::Table::dummy) and the hop cannot open it.
    /// The part still gets its solids when a SECOND relationship reaches a
    /// representation that IS held, which is the case on all three.
    RelatedRepresentationUnresolved,
    /// An item of a representation the hop opened is not a solid.
    ///
    /// **Frequently benign, and measured to be so.** An
    /// `ADVANCED_BREP_SHAPE_REPRESENTATION` routinely lists the placement its
    /// solids are expressed in: `boxy-with-surfacetex.step`'s carries one
    /// `AXIS2_PLACEMENT_3D` beside its single `MANIFOLD_SOLID_BREP`. The reason
    /// exists because the alternative -- dropping the item with no tally -- is
    /// the silence spec 011 T7 exists to end; read the reason, not the bare
    /// [`ShellLoadReport::is_lossless`], when judging a hop.
    ///
    /// On the two Scania files it never fires: all 832 and 254 items of the
    /// reached representations are solids.
    RepresentationItemNotASolid,
}

impl LossReason {
    /// One-line explanation, for messages.
    pub fn description(self) -> &'static str {
        match self {
            Self::FaceUnresolved => "the shell's face reference resolves to no face record",
            Self::SurfaceUnresolved => "the face's surface record could not be resolved",
            Self::SurfaceRefused => "the surface refused conversion to a kernel surface",
            Self::BoundUnresolved => "the face's bound reference resolves to no bound record",
            Self::DegenerateVertexLoop => {
                "the bound is a VERTEX_LOOP: a point boundary at a surface singularity"
            }
            Self::BoundNotALoopWeImplement => {
                "the bound is a loop subtype this loader does not implement"
            }
            Self::EdgeUseUnresolved => "the wire's edge reference has no kept edge",
            Self::EdgeUnresolved => "the edge's children could not be resolved",
            Self::EdgeCurveRefused => "the edge's geometry refused conversion to a curve",
            Self::EdgeEndpointUnresolved => "the edge's start or end vertex has no kept vertex",
            Self::VertexUnresolved => "the vertex's children could not be resolved",
            Self::RelatedRepresentationUnresolved => {
                "the relationship's related representation is not one the table holds"
            }
            Self::RepresentationItemNotASolid => {
                "the related representation lists an item that is not a solid"
            }
        }
    }
}

/// How many, and one worked example.
///
/// The example matters more than it looks: on a real file one reason fires
/// thousands of times with the same text, and the entity id is the only handle a
/// human has for going and looking at the record.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct LossTally {
    /// How many items were lost for this (category, reason).
    pub count: usize,
    /// The STEP entity id of the FIRST loss, when one was known.
    pub first_id: Option<u64>,
    /// The error text of the FIRST loss, when the loss carried one. `None` for
    /// the reasons that are structural rather than a refusal.
    pub first_detail: Option<String>,
}

impl LossTally {
    fn record(&mut self, id: Option<u64>, detail: Option<String>) {
        if self.count == 0 {
            self.first_id = id;
            self.first_detail = detail;
        }
        self.count += 1;
    }
}

/// Listed versus kept, for one [`LossCategory`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CategoryCount {
    /// How many the STEP data asked for.
    pub listed: usize,
    /// How many made it into the converted value.
    pub kept: usize,
}

impl CategoryCount {
    /// `listed - kept`, saturating (they cannot legitimately invert).
    pub fn lost(&self) -> usize { self.listed.saturating_sub(self.kept) }
}

/// What one conversion kept and lost, per category, with reasons.
///
/// Returned alongside the converted value by the `*_reported` methods on
/// [`Table`](crate::step::load::Table). A default-constructed report is the honest
/// description of a conversion that lost nothing.
///
/// Named for shells because shells are what it was built for (spec 011 T7), but
/// it is the load route's ONE loss vocabulary: the part -> solid hop
/// ([`Table::solids_via_shape_relationship`](crate::step::load::Table::solids_via_shape_relationship))
/// reports through the same type, using
/// [`LossCategory::Representation`]/[`LossCategory::Solid`]. A second, parallel
/// report type would have made "did I get everything?" two questions again.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ShellLoadReport {
    counts: BTreeMap<LossCategory, CategoryCount>,
    reasons: BTreeMap<(LossCategory, LossReason), LossTally>,
}

impl ShellLoadReport {
    /// Listed/kept for one category.
    pub fn count(&self, category: LossCategory) -> CategoryCount {
        self.counts.get(&category).copied().unwrap_or_default()
    }

    /// How many items of `category` were lost.
    pub fn lost(&self, category: LossCategory) -> usize { self.count(category).lost() }

    /// How many items were lost across every category.
    pub fn total_lost(&self) -> usize { LossCategory::ALL.iter().map(|c| self.lost(*c)).sum() }

    /// Did this conversion keep every single thing the STEP data listed?
    ///
    /// Note what this does NOT claim: a lossless conversion can still be
    /// geometrically wrong. It claims only that nothing silently vanished.
    pub fn is_lossless(&self) -> bool { self.total_lost() == 0 }

    /// Every (category, reason) that fired, in [`LossCategory::ALL`] order then
    /// reason order.
    pub fn losses(&self) -> impl Iterator<Item = (LossCategory, LossReason, &LossTally)> {
        self.reasons
            .iter()
            .map(|((category, reason), tally)| (*category, *reason, tally))
    }

    /// The tally for one (category, reason) pair; zeroed when it never fired.
    pub fn tally(&self, category: LossCategory, reason: LossReason) -> LossTally {
        self.reasons
            .get(&(category, reason))
            .cloned()
            .unwrap_or_default()
    }

    /// How many items were lost for `reason`, across all categories.
    pub fn lost_for(&self, reason: LossReason) -> usize {
        self.reasons
            .iter()
            .filter(|((_, r), _)| *r == reason)
            .map(|(_, tally)| tally.count)
            .sum()
    }

    /// Fold `other` into `self`. Used to roll a solid's shells up into one
    /// figure; `first_id`/`first_detail` of the receiver win.
    pub fn merge(&mut self, other: &Self) {
        for (category, count) in &other.counts {
            let entry = self.counts.entry(*category).or_default();
            entry.listed += count.listed;
            entry.kept += count.kept;
        }
        for (key, tally) in &other.reasons {
            let entry = self.reasons.entry(*key).or_default();
            if entry.count == 0 {
                entry.first_id = tally.first_id;
                entry.first_detail = tally.first_detail.clone();
            }
            entry.count += tally.count;
        }
    }

    /// Turn loss into a typed error, for callers that want strictness.
    ///
    /// **Opt-in on purpose.** Four in-repo fixtures lose wires today and are
    /// RIGHT to (see [`LossReason::DegenerateVertexLoop`]), so making this the
    /// default would refuse files the kernel handles correctly.
    pub fn require_lossless(&self) -> Result<(), crate::step::load::LoadError> {
        if self.is_lossless() {
            Ok(())
        } else {
            Err(crate::step::load::LoadError::PartialLoss(self.to_string()))
        }
    }

    pub(crate) fn note_listed(&mut self, category: LossCategory, n: usize) {
        self.counts.entry(category).or_default().listed += n;
    }

    pub(crate) fn note_kept(&mut self, category: LossCategory, n: usize) {
        self.counts.entry(category).or_default().kept += n;
    }

    pub(crate) fn note_lost(
        &mut self,
        category: LossCategory,
        reason: LossReason,
        id: Option<u64>,
        detail: Option<String>,
    ) {
        self.reasons
            .entry((category, reason))
            .or_default()
            .record(id, detail);
    }
}

impl fmt::Display for ShellLoadReport {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_lossless() {
            return write!(f, "lossless");
        }
        let mut first = true;
        for category in LossCategory::ALL {
            let count = self.count(category);
            if count.lost() == 0 {
                continue;
            }
            if !first {
                write!(f, "; ")?;
            }
            first = false;
            write!(
                f,
                "{} {} lost of {} listed",
                count.lost(),
                category.plural(),
                count.listed,
            )?;
            for (c, reason, tally) in self.losses() {
                if c != category {
                    continue;
                }
                write!(f, " [{:?} x{}", reason, tally.count)?;
                if let Some(id) = tally.first_id {
                    write!(f, " e.g. #{id}")?;
                }
                if let Some(detail) = &tally.first_detail {
                    write!(f, ": {detail}")?;
                }
                write!(f, "]")?;
            }
        }
        Ok(())
    }
}

/// What the table-building pass swallowed, tallied per STEP entity type.
///
/// Carried by every [`Table`](crate::step::load::Table) as
/// [`Table::entity_report`](crate::step::load::Table::entity_report). Empty means the
/// pass understood, or deliberately filed away, every record it saw.
///
/// Two kinds of swallow are counted, and they are different defects:
///
/// - [`Self::refused`] -- `Deserialize` returned `Err`. The entity type has an
///   arm; the record did not fit the declared holder. The measured population is
///   exporters writing `$` (unset) where a holder declares a non-optional
///   `String`.
/// - [`Self::shape_unexpected`] -- the arm exists but is written as
///   `if let Parameter::List(p) = .. && p.len() == N`, with no `else`. A record
///   with a different arity therefore vanished with neither an error nor a
///   `dummy` entry.
///
/// Records of an entity type that has NO arm are not "swallowed": they are filed
/// in [`Table::dummy`](crate::step::load::Table::dummy) with their name intact, which
/// is already an attributable tally. Look there, not here, for those.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct EntityLoadReport {
    refused: BTreeMap<String, LossTally>,
    shape_unexpected: BTreeMap<String, LossTally>,
}

impl EntityLoadReport {
    /// Did the table-building pass swallow nothing at all?
    pub fn is_empty(&self) -> bool { self.total() == 0 }

    /// How many records were swallowed, both kinds together.
    pub fn total(&self) -> usize { self.total_refused() + self.total_shape_unexpected() }

    /// How many records had a typed `Deserialize` refusal.
    pub fn total_refused(&self) -> usize { self.refused.values().map(|t| t.count).sum() }

    /// How many records vanished in an arity-guarded arm with no `else`.
    pub fn total_shape_unexpected(&self) -> usize {
        self.shape_unexpected.values().map(|t| t.count).sum()
    }

    /// Per-entity-type tallies of typed `Deserialize` refusals, name-ordered.
    ///
    /// The key is the STEP entity type name; a complex (`SUBSUPER`) record is
    /// keyed by its sub-record names joined with `+`.
    pub fn refused(&self) -> impl Iterator<Item = (&str, &LossTally)> {
        self.refused
            .iter()
            .map(|(name, tally)| (name.as_str(), tally))
    }

    /// Per-entity-type tallies of arity-guard vanishings, name-ordered.
    pub fn shape_unexpected(&self) -> impl Iterator<Item = (&str, &LossTally)> {
        self.shape_unexpected
            .iter()
            .map(|(name, tally)| (name.as_str(), tally))
    }

    /// How many records of exactly this entity type were swallowed, either way.
    pub fn swallowed_of(&self, entity_type: &str) -> usize {
        self.refused.get(entity_type).map_or(0, |t| t.count)
            + self
                .shape_unexpected
                .get(entity_type)
                .map_or(0, |t| t.count)
    }

    /// Turn any swallow into a typed error, for callers that want strictness.
    /// Opt-in; the default load path stays `Ok`.
    pub fn require_empty(&self) -> Result<(), crate::step::load::LoadError> {
        if self.is_empty() {
            Ok(())
        } else {
            Err(crate::step::load::LoadError::PartialLoss(self.to_string()))
        }
    }

    pub(crate) fn note_refused(&mut self, entity_type: &str, id: u64, detail: String) {
        self.refused
            .entry(entity_type.to_owned())
            .or_default()
            .record(Some(id), Some(detail));
    }

    pub(crate) fn note_shape_unexpected(&mut self, entity_type: &str, id: u64, detail: String) {
        self.shape_unexpected
            .entry(entity_type.to_owned())
            .or_default()
            .record(Some(id), Some(detail));
    }
}

impl fmt::Display for EntityLoadReport {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_empty() {
            return write!(f, "no entity records swallowed");
        }
        write!(f, "{} entity records swallowed:", self.total())?;
        for (label, entries) in [
            ("refused", &self.refused),
            ("shape-unexpected", &self.shape_unexpected),
        ] {
            for (name, tally) in entries {
                write!(f, " [{label} {name} x{}", tally.count)?;
                if let Some(id) = tally.first_id {
                    write!(f, " e.g. #{id}")?;
                }
                if let Some(detail) = &tally.first_detail {
                    write!(f, ": {detail}")?;
                }
                write!(f, "]")?;
            }
        }
        Ok(())
    }
}

/// The accumulator the conversion route writes through.
///
/// `RefCell` and not `&mut`: one shell conversion is a chain of iterator
/// adaptors whose closures each need to record losses, and several of them are
/// alive at once, so a unique borrow cannot be threaded through. The cell never
/// escapes the conversion call and is never held across a call into user code,
/// so no borrow can overlap.
#[derive(Debug, Default)]
pub(crate) struct LossSink(std::cell::RefCell<ShellLoadReport>);

impl LossSink {
    pub(crate) fn listed(&self, category: LossCategory, n: usize) {
        self.0.borrow_mut().note_listed(category, n);
    }

    pub(crate) fn kept(&self, category: LossCategory, n: usize) {
        self.0.borrow_mut().note_kept(category, n);
    }

    pub(crate) fn lost(
        &self,
        category: LossCategory,
        reason: LossReason,
        id: Option<u64>,
        detail: Option<String>,
    ) {
        self.0.borrow_mut().note_lost(category, reason, id, detail);
    }

    pub(crate) fn into_report(self) -> ShellLoadReport { self.0.into_inner() }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The report must be able to say "nothing was lost" and be believed: a
    /// default report is lossless, and one that only ever recorded KEPT items
    /// stays lossless.
    #[test]
    fn a_report_with_no_losses_is_lossless_and_says_so() {
        let mut report = ShellLoadReport::default();
        assert!(report.is_lossless());
        assert_eq!(report.to_string(), "lossless");
        report.note_listed(LossCategory::Face, 6);
        report.note_kept(LossCategory::Face, 6);
        assert!(report.is_lossless());
        assert!(report.require_lossless().is_ok());
    }

    /// A single lost wire must be visible three ways -- category count, reason
    /// tally, and the `Display` line -- because a caller might reach for any of
    /// them, and a report that is honest in only one of the three is a trap.
    #[test]
    fn one_lost_wire_shows_up_in_the_count_the_reason_and_the_summary() {
        let mut report = ShellLoadReport::default();
        report.note_listed(LossCategory::Wire, 2);
        report.note_kept(LossCategory::Wire, 1);
        report.note_lost(
            LossCategory::Wire,
            LossReason::DegenerateVertexLoop,
            Some(183),
            None,
        );

        assert_eq!(report.lost(LossCategory::Wire), 1);
        assert_eq!(report.count(LossCategory::Wire).listed, 2);
        assert!(!report.is_lossless());
        assert_eq!(
            report.tally(LossCategory::Wire, LossReason::DegenerateVertexLoop),
            LossTally {
                count: 1,
                first_id: Some(183),
                first_detail: None,
            },
        );
        assert_eq!(report.lost_for(LossReason::DegenerateVertexLoop), 1);

        let summary = report.to_string();
        assert!(
            summary.contains("1 wires lost of 2 listed")
                && summary.contains("DegenerateVertexLoop x1")
                && summary.contains("#183"),
            "the summary must carry count, reason and example: {summary}",
        );
        let error = report
            .require_lossless()
            .expect_err("a lossy report must be able to become a typed error");
        assert!(error.to_string().contains("DegenerateVertexLoop"));
    }

    /// Merging is how a solid rolls its shells up. Counts add; the first example
    /// is kept so the handle a human uses does not move.
    #[test]
    fn merging_adds_counts_and_keeps_the_first_example() {
        let mut left = ShellLoadReport::default();
        left.note_listed(LossCategory::Face, 3);
        left.note_kept(LossCategory::Face, 2);
        left.note_lost(
            LossCategory::Face,
            LossReason::SurfaceRefused,
            Some(1),
            Some("left".to_owned()),
        );
        let mut right = ShellLoadReport::default();
        right.note_listed(LossCategory::Face, 5);
        right.note_kept(LossCategory::Face, 4);
        right.note_lost(
            LossCategory::Face,
            LossReason::SurfaceRefused,
            Some(2),
            Some("right".to_owned()),
        );

        left.merge(&right);
        assert_eq!(left.count(LossCategory::Face).listed, 8);
        assert_eq!(left.count(LossCategory::Face).kept, 6);
        assert_eq!(left.lost(LossCategory::Face), 2);
        let tally = left.tally(LossCategory::Face, LossReason::SurfaceRefused);
        assert_eq!(tally.count, 2);
        assert_eq!(tally.first_detail.as_deref(), Some("left"));
    }

    /// The entity report separates the two swallow mechanisms, because they need
    /// different fixes: a refusal is a holder-type question, an unexpected shape
    /// is a missing `else`.
    #[test]
    fn the_entity_report_separates_refusals_from_unexpected_shapes() {
        let mut report = EntityLoadReport::default();
        assert!(report.is_empty());
        assert_eq!(report.to_string(), "no entity records swallowed");

        report.note_refused("PRODUCT_DEFINITION_SHAPE", 7, "invalid type".to_owned());
        report.note_refused("PRODUCT_DEFINITION_SHAPE", 9, "invalid type".to_owned());
        report.note_shape_unexpected("ORIENTED_EDGE", 11, "arity 4".to_owned());

        assert_eq!(report.total(), 3);
        assert_eq!(report.total_refused(), 2);
        assert_eq!(report.total_shape_unexpected(), 1);
        assert_eq!(report.swallowed_of("PRODUCT_DEFINITION_SHAPE"), 2);
        assert_eq!(report.swallowed_of("ORIENTED_EDGE"), 1);
        assert_eq!(report.swallowed_of("PLANE"), 0);
        assert!(report.require_empty().is_err());

        let refused: Vec<_> = report.refused().map(|(name, t)| (name, t.count)).collect();
        assert_eq!(refused, vec![("PRODUCT_DEFINITION_SHAPE", 2)]);
        assert_eq!(
            report.refused().next().and_then(|(_, t)| t.first_id),
            Some(7),
            "the first example id must be the FIRST one seen, not the last",
        );
    }
}
