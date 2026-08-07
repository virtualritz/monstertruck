//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

use super::*;
use monstertruck_core::StableId;

#[test]
fn attribute_crud() {
    let mut attr = Attribute::new();
    let id = StableId::new(1);

    assert!(attr.is_empty());
    attr.set(id, AttributeValue::F32(1.0));
    assert_eq!(attr.len(), 1);
    assert_eq!(attr.get(id), Some(&AttributeValue::F32(1.0)));

    attr.remove(id);
    assert!(attr.is_empty());
}

#[test]
fn element_attributes_named() {
    let mut ea = ElementAttributes::new();
    let id = StableId::new(5);

    ea.set("color", id, AttributeValue::Color([1.0, 0.0, 0.0, 1.0]));
    ea.set("selected", id, AttributeValue::Bool(true));

    assert_eq!(
        ea.get("color", id),
        Some(&AttributeValue::Color([1.0, 0.0, 0.0, 1.0]))
    );
    assert_eq!(ea.get("selected", id), Some(&AttributeValue::Bool(true)));
    assert_eq!(ea.get("missing", id), None);
}

#[test]
fn solid_attributes_empty() {
    let sa = SolidAttributes::new();
    assert!(sa.is_empty());
}

#[test]
fn solid_attributes_not_empty_after_set() {
    let mut sa = SolidAttributes::new();
    sa.faces
        .set("color", StableId::new(1), AttributeValue::Bool(true));
    assert!(!sa.is_empty());
}

#[test]
fn attribute_iter() {
    let mut attr = Attribute::new();
    attr.set(StableId::new(1), AttributeValue::F64(1.0));
    attr.set(StableId::new(2), AttributeValue::F64(2.0));

    let collected: Vec<_> = attr.iter().collect();
    assert_eq!(collected.len(), 2);
}

#[test]
fn element_attributes_remove_attribute() {
    let mut ea = ElementAttributes::new();
    let id = StableId::new(5);
    ea.set("temp", id, AttributeValue::Bool(true));
    assert!(!ea.is_empty());

    let removed = ea.remove_attribute("temp");
    assert!(removed.is_some());
    assert!(ea.is_empty());
}

// -----------------------------------------------------------------------
// Content-hash tests.
// -----------------------------------------------------------------------

#[test]
fn attribute_value_hash_deterministic() {
    let a = AttributeValue::F64(1.0);
    let b = AttributeValue::F64(1.0);
    assert_eq!(a.content_hash64(), b.content_hash64());
}

#[test]
fn attribute_value_different_discriminant() {
    let a = AttributeValue::F64(1.0);
    let b = AttributeValue::F32(1.0);
    assert_ne!(a.content_hash64(), b.content_hash64());
}

#[test]
fn face_attribute_change_changes_hash() {
    let mut attrs = SolidAttributes::new();
    let id = StableId::new(1);
    let before = attrs.content_hash64();
    attrs.faces.set("selected", id, AttributeValue::Bool(true));
    let after = attrs.content_hash64();
    assert_ne!(before, after);
}

#[test]
fn identical_attribute_payloads_match() {
    let id = StableId::new(1);
    let mut a = SolidAttributes::new();
    let mut b = SolidAttributes::new();
    a.faces.set("selected", id, AttributeValue::Bool(true));
    b.faces.set("selected", id, AttributeValue::Bool(true));
    assert_eq!(a.content_hash64(), b.content_hash64());
}

#[test]
fn attribute_insertion_order_irrelevant() {
    let id1 = StableId::new(1);
    let id2 = StableId::new(2);
    let mut a = SolidAttributes::new();
    let mut b = SolidAttributes::new();
    // Insert in opposite order -- BTreeMap sorts by key.
    a.faces.set("x", id1, AttributeValue::Bool(true));
    a.faces.set("x", id2, AttributeValue::Bool(false));
    b.faces.set("x", id2, AttributeValue::Bool(false));
    b.faces.set("x", id1, AttributeValue::Bool(true));
    assert_eq!(a.content_hash64(), b.content_hash64());
}
