use std::fmt::{Debug, Display, Formatter, Result};

use monstertruck_topology::compress::*;

use self::topology::PreStepModel;

const ERR: Result = Err(std::fmt::Error);

#[cfg(feature = "derive")]
pub use monstertruck_derive::{StepFormat, StepLength};

/// display boolean number to step file
#[derive(Clone, Copy, Debug)]
pub struct BooleanDisplay(pub bool);

impl Display for BooleanDisplay {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        match self.0 {
            true => f.write_str(".T."),
            false => f.write_str(".F."),
        }
    }
}

/// display float number to step file
#[derive(Clone, Copy, Debug)]
pub struct FloatDisplay(pub f64);

impl Display for FloatDisplay {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        let FloatDisplay(x) = *self;
        if f64::abs(x) < 1.0e-2 && x != 0.0 {
            f.write_fmt(format_args!("{x:.10E}"))
        } else {
            f.write_fmt(format_args!("{x:?}"))
        }
    }
}

/// display step slice
/// # Examples
/// ```
/// use monstertruck_io::step::save::SliceDisplay;
/// let slice = &[1.0, 2.0, 3.0, 4.0];
/// let display = SliceDisplay(slice);
/// let step_string = display.to_string();
/// assert_eq!(step_string, "(1.0, 2.0, 3.0, 4.0)");
/// ```
#[derive(Clone, Debug)]
pub struct SliceDisplay<'a, T>(pub &'a [T]);

impl Display for SliceDisplay<'_, f64> {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        f.write_str("(")?;
        self.0.iter().enumerate().try_for_each(|(i, x)| {
            if i != 0 {
                f.write_str(", ")?;
            }
            Display::fmt(&FloatDisplay(*x), f)
        })?;
        f.write_str(")")
    }
}

impl Display for SliceDisplay<'_, usize> {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        f.write_str("(")?;
        self.0.iter().enumerate().try_for_each(|(i, x)| {
            if i != 0 {
                f.write_str(", ")?;
            }
            Display::fmt(x, f)
        })?;
        f.write_str(")")
    }
}

impl Display for SliceDisplay<'_, String> {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        f.write_str("(")?;
        self.0.iter().enumerate().try_for_each(|(i, x)| {
            if i != 0 {
                f.write_str(", ")?;
            }
            f.write_fmt(format_args!("'{x}'"))
        })?;
        f.write_str(")")
    }
}

impl<'a> Display for SliceDisplay<'a, SliceDisplay<'a, f64>> {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        f.write_str("(")?;
        self.0.iter().enumerate().try_for_each(|(i, x)| {
            if i != 0 {
                f.write_str(", ")?;
            }
            Display::fmt(x, f)
        })?;
        f.write_str(")")
    }
}

/// display index slice
/// # Examples
/// ```
/// use monstertruck_io::step::save::*;
/// let indices = [1, 10, 100, 1000, 10000];
/// let display = IndexSliceDisplay(indices.into_iter());
/// let step_string = display.to_string();
/// assert_eq!(step_string, "(#1, #10, #100, #1000, #10000)");
/// ```
#[derive(Clone, Debug)]
pub struct IndexSliceDisplay<I>(pub I);

impl<I: Clone + IntoIterator<Item = usize>> Display for IndexSliceDisplay<I> {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        f.write_str("(")?;
        self.0
            .clone()
            .into_iter()
            .enumerate()
            .try_for_each(|(i, idx)| {
                if i != 0 {
                    f.write_fmt(format_args!(", #{idx}"))
                } else {
                    f.write_fmt(format_args!("#{idx}"))
                }
            })?;
        f.write_str(")")
    }
}

impl<I: Clone + IntoIterator<Item = usize>> Display for SliceDisplay<'_, IndexSliceDisplay<I>> {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        f.write_str("(")?;
        self.0.iter().enumerate().try_for_each(|(i, x)| {
            if i != 0 {
                f.write_str(", ")?;
            }
            Display::fmt(x, f)
        })?;
        f.write_str(")")
    }
}

/// STEP-exchange-format counterpart to [`std::fmt::Display`].
///
/// Implementors emit their textual STEP encoding into `f` using `idx` as
/// the entity number (`#idx = ...;`). Sub-entities the impl introduces
/// take subsequent indices; their count is reported via [`StepLength`].
pub trait StepFormat {
    /// Write the STEP encoding of `self` rooted at entity index `idx`.
    fn fmt(&self, idx: usize, f: &mut Formatter<'_>) -> Result;
}

impl<T: StepFormat> StepFormat for &T {
    fn fmt(&self, idx: usize, f: &mut Formatter<'_>) -> Result { StepFormat::fmt(*self, idx, f) }
}

impl<T: StepFormat> StepFormat for Box<T> {
    fn fmt(&self, idx: usize, f: &mut Formatter<'_>) -> Result {
        StepFormat::fmt(self.as_ref(), idx, f)
    }
}

// `DisplayByStep` is the upstream `truck-stepio` trait name. We renamed
// it to `StepFormat` because "display by step" parses as
// "display step-by-step" rather than the intended "format as STEP". The
// alias is kept so external code that imports `DisplayByStep` (e.g. code
// ported from `truck` that pre-dates our rename) keeps compiling against
// `monstertruck`. New code should use `StepFormat`. Slated for removal
// once downstream callers are off the old name.
#[deprecated(since = "0.3.1", note = "renamed to `StepFormat`.")]
pub use StepFormat as DisplayByStep;

/// Display struct for outputting some objects to STEP file format.
#[derive(Clone, Debug)]
pub struct StepDisplay<T> {
    entity: T,
    idx: usize,
}

impl<T> Display for SliceDisplay<'_, StepDisplay<T>>
where StepDisplay<T>: Display
{
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        self.0.iter().try_for_each(|x| Display::fmt(x, f))
    }
}

impl<T> StepDisplay<T> {
    /// constructor
    #[inline]
    pub const fn new(entity: T, idx: usize) -> Self { Self { entity, idx } }
    /// return entity
    #[inline]
    pub const fn entity(&self) -> &T { &self.entity }
    /// return index
    #[inline]
    pub const fn index(&self) -> usize { self.idx }
}

impl<T: StepFormat> Display for StepDisplay<T> {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result { StepFormat::fmt(&self.entity, self.idx, f) }
}

/// Calculate how many lines are used in outputting an object to a STEP file
pub trait StepLength {
    /// Calculate how many lines are used in outputting an object to a STEP file
    fn step_length(&self) -> usize;
}

impl<T: StepLength> StepLength for &T {
    #[inline(always)]
    fn step_length(&self) -> usize { StepLength::step_length(*self) }
}

impl<T: StepLength> StepLength for Box<T> {
    #[inline(always)]
    fn step_length(&self) -> usize { self.as_ref().step_length() }
}

/// Constant numbers of lines for outputting an object to a STEP file.
/// `x.step_length() == X::LENGTH` must always hold.
pub trait ConstStepLength: StepLength {
    /// the number of line
    const LENGTH: usize;
}

impl<T: ConstStepLength> ConstStepLength for &T {
    const LENGTH: usize = T::LENGTH;
}

impl<T: ConstStepLength> ConstStepLength for Box<T> {
    const LENGTH: usize = T::LENGTH;
}

macro_rules! impl_const_step_length {
    ($type: ty, $len: expr $(,<$($gen: ident),*>)?) => {
        impl$(<$($gen),*>)? ConstStepLength for $type {
            const LENGTH: usize = $len;
        }
        impl$(<$($gen),*>)? StepLength for $type {
            #[inline(always)]
            fn step_length(&self) -> usize { <Self as ConstStepLength>::LENGTH }
        }
    };
}

/// Additional information for output to `edge_curve`.
pub trait StepCurve {
    /// the parameter `same_sense`.
    #[inline(always)]
    fn same_sense(&self) -> bool { true }
}

impl<T: StepCurve> StepCurve for &T {
    #[inline(always)]
    fn same_sense(&self) -> bool { (*self).same_sense() }
}

impl<T: StepCurve> StepCurve for Box<T> {
    #[inline(always)]
    fn same_sense(&self) -> bool { self.as_ref().same_sense() }
}

/// Additional information for output to `face_surface`.
pub trait StepSurface {
    /// the parameter `same_sense`.
    #[inline(always)]
    fn same_sense(&self) -> bool { true }
}

impl<T: StepSurface> StepSurface for &T {
    #[inline(always)]
    fn same_sense(&self) -> bool { (*self).same_sense() }
}

impl<T: StepSurface> StepSurface for Box<T> {
    #[inline(always)]
    fn same_sense(&self) -> bool { self.as_ref().same_sense() }
}

/// Describe STEP file header
#[derive(Clone, Debug)]
pub struct StepHeaderDescriptor {
    /// file name
    pub file_name: String,
    /// time stamp
    pub time_stamp: String,
    /// authors
    pub authors: Vec<String>,
    /// organization
    pub organization: Vec<String>,
    /// organization system
    pub organization_system: String,
    /// authorization
    pub authorization: String,
}

#[derive(Clone, Debug)]
struct StepHeader {
    file_name: String,
    time_stamp: String,
    authors: Vec<String>,
    organization: Vec<String>,
    origination_system: String,
    authorization: String,
    schema: String,
}

impl Default for StepHeaderDescriptor {
    fn default() -> Self {
        Self {
            file_name: Default::default(),
            time_stamp: chrono::Utc::now().naive_local().to_string(),
            authors: Default::default(),
            organization: Default::default(),
            organization_system: Default::default(),
            authorization: Default::default(),
        }
    }
}

impl Display for StepHeader {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        let empty_string = [String::new()];
        f.write_fmt(format_args!(
            "HEADER;
FILE_DESCRIPTION(('Shape Data from monstertruck'), '2;1');
FILE_NAME('{file_name}', '{time_stamp}', {authors}, {organization}, 'monstertruck', '{origination_system}', '{authorization}');
FILE_SCHEMA(('{schema}'));
ENDSEC;\n",
            file_name = self.file_name,
            time_stamp = self.time_stamp,
            authors = if self.authors.is_empty() {
                SliceDisplay(&empty_string)
            } else {
                SliceDisplay(&self.authors)
            },
            organization = if self.organization.is_empty() {
                SliceDisplay(&empty_string)
            } else {
                SliceDisplay(&self.organization)
            },
            origination_system = self.origination_system,
            authorization = self.authorization,
            schema = self.schema,
        ))
    }
}

/// SI prefix applied to the base metre length unit in a STEP `SI_UNIT`.
///
/// [`Display`]ing a prefix yields the STEP enumeration token, e.g.
/// [`SiPrefix::Milli`] -> `.MILLI.`. [`SiPrefix::None`] yields `$`, the STEP
/// "unset optional" marker used when the unit is the unscaled metre.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SiPrefix {
    /// `10^18`.
    Exa,
    /// `10^15`.
    Peta,
    /// `10^12`.
    Tera,
    /// `10^9`.
    Giga,
    /// `10^6`.
    Mega,
    /// `10^3`.
    Kilo,
    /// `10^2`.
    Hecto,
    /// `10^1`.
    Deca,
    /// No prefix -- the unscaled base unit.
    None,
    /// `10^-1`.
    Deci,
    /// `10^-2`.
    Centi,
    /// `10^-3`.
    Milli,
    /// `10^-6`.
    Micro,
    /// `10^-9`.
    Nano,
    /// `10^-12`.
    Pico,
    /// `10^-15`.
    Femto,
    /// `10^-18`.
    Atto,
}

impl Display for SiPrefix {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        f.write_str(match self {
            SiPrefix::Exa => ".EXA.",
            SiPrefix::Peta => ".PETA.",
            SiPrefix::Tera => ".TERA.",
            SiPrefix::Giga => ".GIGA.",
            SiPrefix::Mega => ".MEGA.",
            SiPrefix::Kilo => ".KILO.",
            SiPrefix::Hecto => ".HECTO.",
            SiPrefix::Deca => ".DECA.",
            SiPrefix::None => "$",
            SiPrefix::Deci => ".DECI.",
            SiPrefix::Centi => ".CENTI.",
            SiPrefix::Milli => ".MILLI.",
            SiPrefix::Micro => ".MICRO.",
            SiPrefix::Nano => ".NANO.",
            SiPrefix::Pico => ".PICO.",
            SiPrefix::Femto => ".FEMTO.",
            SiPrefix::Atto => ".ATTO.",
        })
    }
}

/// Length unit and distance accuracy written into the
/// `GEOMETRIC_REPRESENTATION_CONTEXT` preamble of a saved model.
///
/// The default preserves the historical output: millimetre lengths and a
/// `distance_accuracy_value` of `1.0E-6`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StepMeasurementContext {
    /// SI prefix of the base metre length unit.
    pub length_prefix: SiPrefix,
    /// `distance_accuracy_value` written into the `UNCERTAINTY_MEASURE_WITH_UNIT`.
    pub distance_accuracy_value: f64,
}

impl Default for StepMeasurementContext {
    fn default() -> Self {
        Self {
            length_prefix: SiPrefix::Milli,
            distance_accuracy_value: 1.0e-6,
        }
    }
}

impl StepMeasurementContext {
    /// STEP `REAL` literal for the distance accuracy value.
    pub(super) fn accuracy(&self) -> StepReal { StepReal(self.distance_accuracy_value) }
}

/// Formats an `f64` as a STEP `REAL` literal: shortest round-tripping form with
/// an uppercase exponent and a mantissa that always carries a decimal point.
#[derive(Clone, Copy, Debug)]
pub(super) struct StepReal(pub f64);

impl Display for StepReal {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        let text = format!("{:E}", self.0);
        let (mantissa, exponent) = match text.split_once('E') {
            Some((mantissa, exponent)) => (mantissa, Some(exponent)),
            None => (text.as_str(), None),
        };
        f.write_str(mantissa)?;
        if !mantissa.contains('.') {
            f.write_str(".0")?;
        }
        match exponent {
            Some(exponent) => f.write_fmt(format_args!("E{exponent}")),
            None => Ok(()),
        }
    }
}

/// Display model with configurations
pub struct StepModel<'a, P, C, S>(PreStepModel<'a, P, C, S>, StepMeasurementContext);

/// Display models with configurations
pub struct StepModels<'a, P, C, S> {
    models: Vec<PreStepModel<'a, P, C, S>>,
    next_idx: usize,
    measurement_context: StepMeasurementContext,
}

impl<P, C, S> Debug for StepModel<'_, P, C, S> {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> Result {
        formatter.debug_tuple("StepModel").finish_non_exhaustive()
    }
}

impl<P, C, S> Debug for StepModels<'_, P, C, S> {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> Result {
        formatter
            .debug_struct("StepModels")
            .field("count", &self.models.len())
            .field("next_idx", &self.next_idx)
            .finish()
    }
}

/// Display struct for outputting STEP file format with header.
#[derive(Clone, Debug)]
pub struct CompleteStepDisplay<T> {
    display: T,
    header: StepHeader,
}

impl<T: Display> Display for CompleteStepDisplay<T> {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        f.write_fmt(format_args!(
            "ISO-10303-21;\n{}DATA;\n{}ENDSEC;\nEND-ISO-10303-21;\n",
            self.header, self.display,
        ))
    }
}

impl<T> CompleteStepDisplay<T> {
    /// constructor
    #[inline]
    pub fn new(display: T, header: StepHeaderDescriptor) -> Self {
        CompleteStepDisplay {
            display,
            header: StepHeader {
                file_name: header.file_name,
                time_stamp: header.time_stamp,
                authors: header.authors,
                organization: header.organization,
                origination_system: header.organization_system,
                authorization: header.authorization,
                // The schema identifier must name the EXPRESS schema the file
                // populates, not an ISO 10303 part number. The data section
                // declares the `automotive_design` (AP214) application
                // protocol, whose schema is `AUTOMOTIVE_DESIGN`.
                schema: "AUTOMOTIVE_DESIGN".to_string(),
            },
        }
    }
}

mod assembly;
mod geometry;
mod topology;
pub use assembly::StepDesign;
pub use geometry::{MatrixAsAxis, VectorAsDirection};
