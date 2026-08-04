//! STEP file import and export.
//!
//! # Examples
//!
//! ```
//! use monstertruck_step::load::{*, step_geometry::*};
//!
//! // Parse a STEP file. Any `&[u8]` will do; this uses an in-repo fixture so
//! // the example is executed rather than merely type-checked.
//! let bytes = include_bytes!(concat!(
//!     env!("CARGO_MANIFEST_DIR"),
//!     "/../resources/step/occt-cube.step",
//! ));
//! let table = Table::from_step_bytes(bytes).unwrap();
//!
//! // Extract a shell and convert it to topology.
//! let step_shell = table.shell.values().next().unwrap();
//! let compressed = table.to_compressed_shell(step_shell).unwrap();
//! ```

#![cfg_attr(not(debug_assertions), deny(warnings))]
#![deny(clippy::all, rust_2018_idioms)]
#![warn(
    missing_docs,
    missing_debug_implementations,
    trivial_casts,
    trivial_numeric_casts,
    unsafe_code,
    unstable_features,
    unused_import_braces,
    unused_qualifications
)]

/// STEP file loading (import/parsing).
///
/// # Example
/// ```
/// # fn main() -> anyhow::Result<()> {
/// use monstertruck_step::load::{*, step_geometry::*};
/// use step_p21::tables::EntityTable;
/// // read file
/// let step_string = include_str!(concat!(
///     env!("CARGO_MANIFEST_DIR"),
///     "/../resources/step/occt-cube.step",
/// ));
/// // parse step file
/// let exchange = step_p21::parser::parse(&step_string)?;
/// // convert the parsing results to a Rust struct
/// let table = Table::from_data_section(&exchange.data[0]);
/// // get `CartesianPoint` registered in #102
/// let step_point = EntityTable::<CartesianPointHolder>::get_owned(&table, 102)?;
/// // convert `CartesianPoint` in STEP to `Point3` in cgmath
/// let cgmath_point = Point3::from(&step_point);
/// // check parse result
/// assert_eq!(cgmath_point, Point3::new(0.0, 10.0, 0.0));
/// # Ok(())
/// # }
/// ```
/// Types shared between the [`load`] and [`save`] sides of STEP I/O.
pub mod common;
#[cfg(feature = "load")]
pub mod load;
/// STEP file saving (export/formatting).
pub mod save;

#[doc(hidden)]
#[macro_export]
macro_rules! impl_from {
	($(impl From<&$refed: ty> for $converted: ty {
		$from_func: item
	})*) => {
		$(impl From<&$refed> for $converted {
			$from_func
		}
		impl From<$refed> for $converted {
			fn from(x: $refed) -> Self { Self::from(&x) }
		})*
	};
}

#[doc(hidden)]
#[macro_export]
macro_rules! impl_try_from {
	($(impl TryFrom<&$refed: ty> for $converted: ty {
		$try_from_func: item
	})*) => {
		$(impl TryFrom<&$refed> for $converted {
            type Error = ExpressParseError;
			$try_from_func
		}
		impl TryFrom<$refed> for $converted {
            type Error = ExpressParseError;
            fn try_from(x: $refed) -> Result<Self, ExpressParseError> { Self::try_from(&x) }
		})*
	};
}
