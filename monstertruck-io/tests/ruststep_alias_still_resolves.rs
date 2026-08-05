//! The `ruststep` re-export path must keep resolving after the rename.
//!
//! `monstertruck_io::step::load::ruststep` was the parser re-export for as long as
//! the parser was upstream `ruststep`. Renaming it to `step_p21` without an alias
//! silently breaks every `load::ruststep::...` path a consumer wrote, and a
//! consumer has no way to see that coming from a version bump. The alias is
//! deprecated, not removed, so the break is a warning rather than a compile
//! error.
//!
//! This is a type-identity check: it passes only if both paths name the SAME
//! type, so an alias pointing at something else would not compile.

#![cfg(feature = "load")]
#![allow(deprecated)]

use monstertruck_io::step::load::{ruststep, step_p21};

/// Compiles only if `ruststep::ast::Name` and `step_p21::ast::Name` are one type.
fn _the_two_paths_name_one_type(name: ruststep::ast::Name) -> step_p21::ast::Name { name }

#[test]
fn the_deprecated_ruststep_path_still_resolves() {
    // Reached through the OLD path, compared against a value built the new way.
    let old: ruststep::ast::Name = ruststep::ast::Name::Entity(7);
    let new: step_p21::ast::Name = step_p21::ast::Name::Entity(7);
    assert_eq!(old, new, "the alias must reach the same parser, not a copy");
}
