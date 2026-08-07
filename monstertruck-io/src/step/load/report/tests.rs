//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

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
