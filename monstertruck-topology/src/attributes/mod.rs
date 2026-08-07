//! Per-element attribute storage for topology elements.
//!
//! Named attributes store sparse per-element data keyed by [`StableId`].
//! Attribute values are typed via [`AttributeValue`].

use std::collections::BTreeMap;
use std::hash::Hasher;

use monstertruck_core::{DeterministicContentHash, StableId};
use serde::{Deserialize, Serialize};

/// Typed attribute values.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)
)]
pub enum AttributeValue {
    /// 32-bit floating point.
    F32(f32),
    /// 64-bit floating point.
    F64(f64),
    /// 3D vector (f64).
    Vec3([f64; 3]),
    /// RGBA color (f32).
    Color([f32; 4]),
    /// Boolean flag.
    Bool(bool),
    /// String value.
    String(String),
    /// A set of StableIds (for selection-set attributes).
    IdSet(Vec<StableId>),
}

/// A single named attribute: sparse per-element data.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)
)]
pub struct Attribute {
    values: BTreeMap<StableId, AttributeValue>,
}

impl Attribute {
    /// Creates a new empty attribute.
    pub fn new() -> Self { Self::default() }

    /// Gets the value for a given element.
    pub fn get(&self, id: StableId) -> Option<&AttributeValue> { self.values.get(&id) }

    /// Sets the value for a given element.
    pub fn set(&mut self, id: StableId, value: AttributeValue) { self.values.insert(id, value); }

    /// Removes the value for a given element, returning it if present.
    pub fn remove(&mut self, id: StableId) -> Option<AttributeValue> { self.values.remove(&id) }

    /// Returns whether a value exists for the given element.
    pub fn contains(&self, id: StableId) -> bool { self.values.contains_key(&id) }

    /// Returns an iterator over all `(StableId, &AttributeValue)` pairs.
    pub fn iter(&self) -> impl Iterator<Item = (StableId, &AttributeValue)> + '_ {
        self.values.iter().map(|(&k, v)| (k, v))
    }

    /// Returns the number of elements with values.
    pub fn len(&self) -> usize { self.values.len() }

    /// Returns whether no elements have values.
    pub fn is_empty(&self) -> bool { self.values.is_empty() }

    /// Removes all values.
    pub fn clear(&mut self) { self.values.clear(); }
}

/// All named attributes for one element type.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)
)]
pub struct ElementAttributes {
    attrs: BTreeMap<String, Attribute>,
}

impl ElementAttributes {
    /// Creates a new empty element attributes container.
    pub fn new() -> Self { Self::default() }

    /// Gets a value for a named attribute on a given element.
    pub fn get(&self, name: &str, id: StableId) -> Option<&AttributeValue> {
        self.attrs.get(name)?.get(id)
    }

    /// Sets a value for a named attribute on a given element.
    pub fn set(&mut self, name: &str, id: StableId, value: AttributeValue) {
        self.attrs
            .entry(name.to_string())
            .or_default()
            .set(id, value);
    }

    /// Removes a value for a named attribute on a given element.
    pub fn remove(&mut self, name: &str, id: StableId) -> Option<AttributeValue> {
        self.attrs.get_mut(name)?.remove(id)
    }

    /// Returns a reference to the named attribute, if it exists.
    pub fn attribute(&self, name: &str) -> Option<&Attribute> { self.attrs.get(name) }

    /// Returns a mutable reference to the named attribute, creating it if absent.
    pub fn attribute_mut(&mut self, name: &str) -> &mut Attribute {
        self.attrs.entry(name.to_string()).or_default()
    }

    /// Removes an entire named attribute, returning it if present.
    pub fn remove_attribute(&mut self, name: &str) -> Option<Attribute> { self.attrs.remove(name) }

    /// Returns an iterator over all attribute names.
    pub fn names(&self) -> impl Iterator<Item = &str> { self.attrs.keys().map(String::as_str) }

    /// Returns whether there are no named attributes.
    pub fn is_empty(&self) -> bool { self.attrs.is_empty() }
}

/// Complete attribute storage for a Solid.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(
    feature = "rkyv",
    derive(rkyv::Archive, rkyv::Serialize, rkyv::Deserialize)
)]
pub struct SolidAttributes {
    /// Per-vertex attributes.
    pub vertices: ElementAttributes,
    /// Per-edge attributes.
    pub edges: ElementAttributes,
    /// Per-face attributes.
    pub faces: ElementAttributes,
    /// Per-trim-curve attributes.
    pub trim_curves: ElementAttributes,
    /// Per-iso-parameter attributes.
    pub iso_parameters: ElementAttributes,
}

impl SolidAttributes {
    /// Creates a new empty attribute store.
    pub fn new() -> Self { Self::default() }

    /// Returns whether all element attribute stores are empty.
    pub fn is_empty(&self) -> bool {
        self.vertices.is_empty()
            && self.edges.is_empty()
            && self.faces.is_empty()
            && self.trim_curves.is_empty()
            && self.iso_parameters.is_empty()
    }
}

// ---------------------------------------------------------------------------
// Deterministic content hashing.
// ---------------------------------------------------------------------------

impl DeterministicContentHash for AttributeValue {
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        // Write discriminant tag first.
        match self {
            Self::F32(v) => {
                state.write_u8(0);
                v.content_hash(state);
            }
            Self::F64(v) => {
                state.write_u8(1);
                v.content_hash(state);
            }
            Self::Vec3(v) => {
                state.write_u8(2);
                v.content_hash(state);
            }
            Self::Color(v) => {
                state.write_u8(3);
                v.content_hash(state);
            }
            Self::Bool(v) => {
                state.write_u8(4);
                v.content_hash(state);
            }
            Self::String(v) => {
                state.write_u8(5);
                v.content_hash(state);
            }
            Self::IdSet(v) => {
                state.write_u8(6);
                v.content_hash(state);
            }
        }
    }
}

impl DeterministicContentHash for Attribute {
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        // BTreeMap iterates in key order -- deterministic.
        self.values.content_hash(state);
    }
}

impl DeterministicContentHash for ElementAttributes {
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        // BTreeMap iterates in key order -- deterministic.
        self.attrs.content_hash(state);
    }
}

impl DeterministicContentHash for SolidAttributes {
    fn content_hash<H: Hasher>(&self, state: &mut H) {
        // Hash in fixed element-kind order.
        self.vertices.content_hash(state);
        self.edges.content_hash(state);
        self.faces.content_hash(state);
        self.trim_curves.content_hash(state);
        self.iso_parameters.content_hash(state);
    }
}

#[cfg(test)]
mod tests;
