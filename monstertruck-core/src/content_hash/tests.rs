//! Unit tests for the parent module.
//!
//! Split out of the module file so the source stays readable; the module
//! path is unchanged.

use super::*;

#[test]
fn same_input_same_hash() {
    assert_eq!(1_u64.content_hash64(), 1_u64.content_hash64());
}

#[test]
fn different_input_different_hash() {
    assert_ne!(1_u64.content_hash64(), 2_u64.content_hash64());
}

#[test]
fn bool_hashing() {
    assert_ne!(true.content_hash64(), false.content_hash64());
}

#[test]
fn string_hashing() {
    assert_eq!("hello".content_hash64(), "hello".content_hash64());
    assert_ne!("hello".content_hash64(), "world".content_hash64());
}

#[test]
fn vec_hashing() {
    let a = vec![1_u64, 2, 3];
    let b = vec![1_u64, 2, 3];
    let c = vec![1_u64, 2, 4];
    assert_eq!(a.content_hash64(), b.content_hash64());
    assert_ne!(a.content_hash64(), c.content_hash64());
}

#[test]
fn option_hashing() {
    let a: Option<u64> = Some(42);
    let b: Option<u64> = Some(42);
    let c: Option<u64> = None;
    assert_eq!(a.content_hash64(), b.content_hash64());
    assert_ne!(a.content_hash64(), c.content_hash64());
}

#[test]
fn f64_bit_pattern_hashing() {
    assert_eq!(1.0_f64.content_hash64(), 1.0_f64.content_hash64());
    assert_ne!(1.0_f64.content_hash64(), 1.0000000001_f64.content_hash64());
}

#[test]
fn point3_hashing() {
    let a = Point3::new(1.0, 2.0, 3.0);
    let b = Point3::new(1.0, 2.0, 3.0);
    let c = Point3::new(1.0, 2.0, 4.0);
    assert_eq!(a.content_hash64(), b.content_hash64());
    assert_ne!(a.content_hash64(), c.content_hash64());
}

#[test]
fn stable_id_hashing() {
    let a = StableId::new(1);
    let b = StableId::new(1);
    let c = StableId::new(2);
    assert_eq!(a.content_hash64(), b.content_hash64());
    assert_ne!(a.content_hash64(), c.content_hash64());
}

#[test]
fn btreemap_hashing() {
    let a = BTreeMap::from([(1_u64, 10_u64), (2, 20)]);
    let b = BTreeMap::from([(1_u64, 10_u64), (2, 20)]);
    let c = BTreeMap::from([(1_u64, 10_u64), (2, 21)]);
    assert_eq!(a.content_hash64(), b.content_hash64());
    assert_ne!(a.content_hash64(), c.content_hash64());
}

#[test]
fn tuple_hashing() {
    assert_eq!(
        (1_u64, 2_u64).content_hash64(),
        (1_u64, 2_u64).content_hash64()
    );
    assert_ne!(
        (1_u64, 2_u64).content_hash64(),
        (1_u64, 3_u64).content_hash64()
    );
}
