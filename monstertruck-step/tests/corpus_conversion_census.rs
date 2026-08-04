//! Face-level CONVERSION census -- spec 011 Phase 0.
//!
//! `corpus_load.rs` answers "does the table parse?". This file answers the
//! harder question: **of the `ADVANCED_FACE`s a file contains, how many reach a
//! `ModelingSurface` the kernel can actually represent?** The two numbers are
//! unrelated -- a file can table-parse perfectly and still lose faces, because
//! the load path drops a face whose surface fails to resolve or convert and
//! returns `Ok` anyway (`load/convert.rs`, `shell_faces`/`shell_trimmed_faces`:
//! `.map_err(|e| eprintln!("{e}")).ok()?`).
//!
//! The census test REPORTS. Its only assertions are internal-consistency ones
//! (every face lands in exactly one bucket, every solid in exactly one assembly
//! bucket) -- a census that asserted conversion success would just be a wish.
//! The three cheap tests beside it are different: they pin specific behaviour
//! this census MEASURED, so the numbers above cannot rot unnoticed.
//!
//! Read-only: no production code is touched, no instrumentation is compiled in.
//! The classification is reconstructed from the public `Table` maps plus the
//! same two conversion calls the loader itself makes.
//!
//! Conversion has four possible outcomes here, not three. Besides converting and
//! refusing typed, it can **panic** -- measured, see `install_census_panic_hook`
//! -- and a boundary wire can vanish with no message at all. Both get a column.
//!
//! Corpus-gated and `#[ignore]`d -- ~1 GB of input. Run with:
//!
//! ```text
//! cargo nextest run -p monstertruck-step --run-ignored all \
//!     -E 'test(corpus_conversion_census)' --no-capture
//! ```

use monstertruck_step::load::{SurfaceAny, Table, step_geometry::Surface};
use std::cell::{Cell, RefCell};
use std::collections::BTreeMap;
use std::panic::AssertUnwindSafe;
use std::path::{Path, PathBuf};
use std::str::FromStr;
use step_p21::{
    ast::{DataSection, EntityInstance, Name, SubSuperRecord},
    tables::{IntoOwned, PlaceHolder},
};

// ------------------------------------------------------- panic containment
//
// Measured 2026-07-29: conversion does not merely refuse on some real-world
// input, it PANICS (`Torus::new`, `monstertruck-geometry/src/specifieds/
// torus.rs:9`, on a negative major radius). A census that aborts on the first
// such record can only ever report the first one, so panics are caught and
// counted as their own bucket. Nothing is suppressed: the message and location
// are recorded and printed per class.

thread_local! {
    /// True while inside a deliberately-caught conversion, so the hook stays
    /// quiet. Outside one -- e.g. a genuine assertion failure in this test --
    /// the hook prints normally and nothing is hidden.
    static CATCHING: Cell<bool> = const { Cell::new(false) };
    static LAST_PANIC: RefCell<Option<String>> = const { RefCell::new(None) };
}

fn install_census_panic_hook() {
    std::panic::set_hook(Box::new(|info| {
        if CATCHING.get() {
            let payload = info
                .payload()
                .downcast_ref::<&str>()
                .map(|s| (*s).to_owned())
                .or_else(|| info.payload().downcast_ref::<String>().cloned())
                .unwrap_or_else(|| "<non-string panic payload>".to_owned());
            let at = info
                .location()
                .map(|l| format!(" at {}:{}", l.file(), l.line()))
                .unwrap_or_default();
            LAST_PANIC.with(|slot| *slot.borrow_mut() = Some(format!("{payload}{at}")));
        } else {
            eprintln!("{info}");
        }
    }));
}

/// Run `f`, converting a panic into `Err(message)`. `AssertUnwindSafe` is sound
/// here because every closure passed in only READS the table.
fn catching<T>(f: impl FnOnce() -> T) -> Result<T, String> {
    CATCHING.set(true);
    let caught = std::panic::catch_unwind(AssertUnwindSafe(f));
    CATCHING.set(false);
    caught.map_err(|_| {
        LAST_PANIC
            .with(|slot| slot.borrow_mut().take())
            .unwrap_or_else(|| "<panic>".to_owned())
    })
}

// ---------------------------------------------------------------- discovery

const CORPUS_FILES: [&str; 8] = [
    "ROTOR-201NAL-Z7.STEP",
    "Rocky_House.stp",
    "Cruise_Assembly.stp",
    "UMC-500_SS_Solid_Model_2019-06_r1.stp",
    "Ai-14R.stp",
    "NissanGT-R.STEP",
    "Scania-8x4.stp",
    "Scania-Engine-V8-XT-Turbo.step",
];

/// Same discovery contract as `corpus_load.rs`: an explicit env override, else a
/// home-relative default, and `None` (-> SKIP) when neither is a directory.
fn corpus_root() -> Option<PathBuf> {
    std::env::var_os("MONSTERTRUCK_STEP_CORPUS")
        .map(PathBuf::from)
        .or_else(|| {
            std::env::var_os("HOME").map(|h| Path::new(&h).join("code/step-corpus/bigassy"))
        })
        .filter(|p| p.is_dir())
}

fn repo_fixtures() -> Vec<PathBuf> {
    let manifest = Path::new(env!("CARGO_MANIFEST_DIR"));
    [
        manifest.join("../resources/step"),
        manifest.join("tests/fixtures/real-world"),
    ]
    .iter()
    .filter_map(|dir| std::fs::read_dir(dir).ok())
    .flatten()
    .filter_map(|entry| entry.ok().map(|e| e.path()))
    .filter(|p| {
        p.extension()
            .and_then(|e| e.to_str())
            .is_some_and(|e| e.eq_ignore_ascii_case("step") || e.eq_ignore_ascii_case("stp"))
    })
    .collect()
}

// ------------------------------------------------------------ surface class

/// The surface classes the census reports on, named as the STEP entity is
/// named. `Dummy` carries the entity name verbatim for records the table's
/// `push_instance` did not recognise (its `_ =>` arm files them under
/// `Table::dummy`), which is where an unmodelled subtype ends up.
///
/// `DEGENERATE_TOROIDAL_SURFACE` used to be the one surface class in that state.
/// Since spec 011 T1 it has a schema variant and a table map, so it reports as
/// `Typed` -- and its refusal names the class instead of a lookup miss.
#[derive(Clone, PartialEq, Eq, PartialOrd, Ord, Debug)]
enum Class {
    /// Present in a typed surface map of the table.
    Typed(&'static str),
    /// Present in the table only as an unrecognised record, name included.
    Unrecognised(String),
    /// No record with this id exists anywhere in the table.
    Absent,
    /// The face's `face_geometry` was not an entity reference at all.
    NotAReference,
}

impl Class {
    fn label(&self) -> String {
        match self {
            Self::Typed(name) => (*name).to_owned(),
            Self::Unrecognised(name) => format!("{name} (unrecognised)"),
            Self::Absent => "<no such record>".to_owned(),
            Self::NotAReference => "<inline, not a reference>".to_owned(),
        }
    }
}

/// Which table map holds `id`, if any. Order is presentation order, not
/// semantics -- the maps are disjoint by construction (`push_instance` inserts
/// each id into exactly one).
fn typed_class(table: &Table, id: u64) -> Option<&'static str> {
    let probes: [(&'static str, bool); 13] = [
        ("PLANE", table.plane.contains_key(&id)),
        (
            "CYLINDRICAL_SURFACE",
            table.cylindrical_surface.contains_key(&id),
        ),
        ("CONICAL_SURFACE", table.conical_surface.contains_key(&id)),
        (
            "SPHERICAL_SURFACE",
            table.spherical_surface.contains_key(&id),
        ),
        ("TOROIDAL_SURFACE", table.toroidal_surface.contains_key(&id)),
        (
            "DEGENERATE_TOROIDAL_SURFACE",
            table.degenerate_toroidal_surface.contains_key(&id),
        ),
        (
            "B_SPLINE_SURFACE_WITH_KNOTS",
            table.b_spline_surface_with_knots.contains_key(&id),
        ),
        (
            "RATIONAL_B_SPLINE_SURFACE",
            table.rational_b_spline_surface.contains_key(&id),
        ),
        ("BEZIER_SURFACE", table.bezier_surface.contains_key(&id)),
        ("UNIFORM_SURFACE", table.uniform_surface.contains_key(&id)),
        (
            "QUASI_UNIFORM_SURFACE",
            table.quasi_uniform_surface.contains_key(&id),
        ),
        (
            "SURFACE_OF_LINEAR_EXTRUSION",
            table.surface_of_linear_extrusion.contains_key(&id),
        ),
        (
            "SURFACE_OF_REVOLUTION",
            table.surface_of_revolution.contains_key(&id),
        ),
    ];
    probes
        .into_iter()
        .find(|(_, hit)| *hit)
        .map(|(name, _)| name)
}

/// `Table::dummy` stores `format!("{record:?}")`, whose derived `Debug` puts the
/// entity name in the first quoted field. Extracting it is a diagnostic
/// convenience, not a contract -- an unparseable shape degrades to `?`.
fn unrecognised_name(record: &str) -> String { record.split('"').nth(1).unwrap_or("?").to_owned() }

fn class_of(table: &Table, id: u64) -> Class {
    typed_class(table, id).map(Class::Typed).unwrap_or_else(|| {
        table
            .dummy
            .get(&id)
            .map(|d| Class::Unrecognised(unrecognised_name(&d.record)))
            .unwrap_or(Class::Absent)
    })
}

// -------------------------------------------------------------- face fates

/// Exactly one of these per `ADVANCED_FACE`/`FACE_SURFACE` in the table.
#[derive(Clone, PartialEq, Eq, PartialOrd, Ord, Debug)]
enum Fate {
    /// Deserialised AND `Surface::try_from(&SurfaceAny)` succeeded.
    Converted,
    /// Resolved to a `SurfaceAny`, but the conversion to the modeling surface
    /// refused. Carries the error text.
    RefusedConversion(String),
    /// `PlaceHolder::into_owned` refused. Carries the error text. The class
    /// tells you whether the record was unrecognised, absent, or present but
    /// with an unresolvable child.
    RefusedResolution(String),
    /// Conversion PANICKED. Not a refusal -- an unwind. In a caller that does
    /// not catch, this aborts the load of the whole file.
    Panicked(String),
}

/// Replace `#<digits>` with `#<id>` so N refusals that differ only in entity id
/// group into one reported row. Without this, Rocky_House alone emits 156 lines
/// that say the same thing, which buries every other finding.
fn without_ids(message: &str) -> String {
    let mut out = String::with_capacity(message.len());
    let mut chars = message.chars().peekable();
    while let Some(c) = chars.next() {
        out.push(c);
        if c == '#' && chars.peek().is_some_and(char::is_ascii_digit) {
            while chars.peek().is_some_and(char::is_ascii_digit) {
                chars.next();
            }
            out.push_str("<id>");
        }
    }
    out
}

fn fate_of(table: &Table, holder: &monstertruck_step::load::FaceSurfaceHolder) -> Fate {
    let resolved = catching(|| holder.face_geometry.clone().into_owned(table));
    match resolved {
        Err(panic) => Fate::Panicked(panic),
        Ok(Err(e)) => Fate::RefusedResolution(e.to_string()),
        Ok(Ok(surface_any)) => surface_any_fate(&surface_any),
    }
}

fn surface_any_fate(surface_any: &SurfaceAny) -> Fate {
    match catching(|| Surface::try_from(surface_any)) {
        Err(panic) => Fate::Panicked(panic),
        Ok(Ok(_)) => Fate::Converted,
        Ok(Err(e)) => Fate::RefusedConversion(e.to_string()),
    }
}

fn face_geometry_id(holder: &monstertruck_step::load::FaceSurfaceHolder) -> Option<u64> {
    match &holder.face_geometry {
        PlaceHolder::Ref(Name::Entity(id)) => Some(*id),
        _ => None,
    }
}

// -------------------------------------------------------------- accumulator

#[derive(Default)]
struct ClassTally {
    converted: usize,
    refused_conversion: BTreeMap<String, usize>,
    refused_resolution: BTreeMap<String, usize>,
    panicked: BTreeMap<String, usize>,
}

impl ClassTally {
    fn panics(&self) -> usize { self.panicked.values().sum() }
    fn total(&self) -> usize {
        self.converted
            + self.refused_conversion.values().sum::<usize>()
            + self.refused_resolution.values().sum::<usize>()
            + self.panics()
    }
    fn refused(&self) -> usize { self.total() - self.converted - self.panics() }
}

#[derive(Default)]
struct FileCensus {
    faces_in_table: usize,
    by_class: BTreeMap<Class, ClassTally>,
    /// `face_geometry` -> `Converted?`, memoised for the assembly pass.
    converted: std::collections::HashMap<u64, bool>,
}

fn census_faces(table: &Table) -> FileCensus {
    let mut census = FileCensus {
        faces_in_table: table.face_surface.len(),
        ..Default::default()
    };
    table.face_surface.iter().for_each(|(&face_id, holder)| {
        let class = match face_geometry_id(holder) {
            Some(id) => class_of(table, id),
            None => Class::NotAReference,
        };
        let fate = fate_of(table, holder);
        census.converted.insert(face_id, fate == Fate::Converted);
        let tally = census.by_class.entry(class).or_default();
        match fate {
            Fate::Converted => tally.converted += 1,
            Fate::RefusedConversion(e) => {
                *tally.refused_conversion.entry(without_ids(&e)).or_default() += 1
            }
            Fate::RefusedResolution(e) => {
                *tally.refused_resolution.entry(without_ids(&e)).or_default() += 1
            }
            Fate::Panicked(e) => *tally.panicked.entry(without_ids(&e)).or_default() += 1,
        }
    });
    census
}

// ------------------------------------------- swallowed entity-level refusals
//
// `Table::from_iter` is
//
//     iter.for_each(|instance| res.push_instance(instance).unwrap_or_else(|e| eprintln!("{e}")))
//
// so a record whose `Deserialize` refuses is printed to stderr and then DROPPED
// -- the entity never enters the table and `from_step_bytes` still returns `Ok`.
// From the caller's side that is invisible: the only evidence is a stderr line
// nobody reads, and the affected entity simply does not exist afterwards.
//
// The census reproduces `from_iter` EXACTLY -- same public `push_instance`, same
// order -- but records what got swallowed instead of printing it. No production
// change is involved; `push_instance` is already `pub`.

/// One swallowed refusal, aggregated: how many, and up to three example ids so a
/// finding is traceable back to a byte offset in the file.
#[derive(Default)]
struct Refusal {
    count: usize,
    sample_ids: Vec<u64>,
}

type Refusals = BTreeMap<(String, String), Refusal>;

fn entity_name(instance: &EntityInstance) -> String {
    match instance {
        EntityInstance::Simple { record, .. } => record.name.clone(),
        EntityInstance::Complex {
            subsuper: SubSuperRecord(records),
            ..
        } => records
            .iter()
            .map(|r| r.name.as_str())
            .collect::<Vec<_>>()
            .join("+"),
    }
}

fn entity_id(instance: &EntityInstance) -> u64 {
    match instance {
        EntityInstance::Simple { id, .. } | EntityInstance::Complex { id, .. } => *id,
    }
}

/// Decode exactly as [`Table::from_step_bytes`] does -- UTF-8 where possible,
/// total ISO-8859-1 fallback where not -- then build the table entity by entity,
/// keeping the refusals.
fn table_with_refusals(bytes: &[u8]) -> Result<(Table, Refusals), String> {
    let text: String = match std::str::from_utf8(bytes) {
        Ok(text) => text.to_owned(),
        Err(_) => bytes.iter().map(|&b| b as char).collect(),
    };
    let exchange = step_p21::parser::parse(&text).map_err(|e| e.to_string())?;
    let section = exchange.data.first().ok_or("no DATA section")?;
    let mut table = Table::default();
    let mut refusals = Refusals::new();
    section.entities.iter().for_each(|instance| {
        if let Err(e) = table.push_instance(instance) {
            let key = (entity_name(instance), without_ids(&e.to_string()));
            let entry = refusals.entry(key).or_default();
            entry.count += 1;
            if entry.sample_ids.len() < 3 {
                entry.sample_ids.push(entity_id(instance));
            }
        }
    });
    Ok((table, refusals))
}

// -------------------------------------------------- raw entity-name counter

/// One pass over the raw bytes counting maximal `[A-Z][A-Z0-9_]*` runs that are
/// followed -- **across any intervening whitespace** -- by `(`.
///
/// Two traps this deliberately steps around, both of which silently zero a
/// whole file if you get them wrong:
///
/// 1. **Encoding.** `Ai-14R.stp` is ISO-8859, so anything insisting on UTF-8
///    (or a `grep` without `-a`) reports zero for every entity in it. Hence
///    byte-level.
/// 2. **Whitespace before the argument list.** `NissanGT-R.STEP`,
///    `ROTOR-201NAL-Z7.STEP` and `UMC-500_SS_...stp` are space-separated --
///    `#5 = ADVANCED_FACE ( 'NONE' , ( #636919 ) , #1096349 , .F. ) ;`. A scan
///    for the literal `ADVANCED_FACE(` finds NOTHING in those three files, and
///    so does `grep -aoE 'ADVANCED_FACE\('`. The formatting is legal ISO
///    10303-21 and the tokenizer is entirely happy with it; only a naive
///    counter is fooled.
///
/// This is an OCCURRENCE count, not `grep -c`'s lines-with-a-match.
fn raw_entity_counts(bytes: &[u8]) -> BTreeMap<String, usize> {
    let ident = |b: u8| b.is_ascii_uppercase() || b.is_ascii_digit() || b == b'_';
    let mut counts = BTreeMap::<String, usize>::new();
    let mut i = 0usize;
    while i < bytes.len() {
        if bytes[i].is_ascii_uppercase() {
            let start = i;
            while i < bytes.len() && ident(bytes[i]) {
                i += 1;
            }
            let end = i;
            while i < bytes.len() && bytes[i].is_ascii_whitespace() {
                i += 1;
            }
            if bytes.get(i) == Some(&b'(') {
                let name = String::from_utf8_lossy(&bytes[start..end]).into_owned();
                *counts.entry(name).or_default() += 1;
            }
        } else {
            i += 1;
        }
    }
    counts
}

/// Reported alongside the measured table counts. `BREP_WITH_VOIDS` is here
/// because it shares `Table::manifold_solid_brep` with `MANIFOLD_SOLID_BREP`, so
/// the solid count only reconciles when both are visible. `VERTEX_LOOP` is here
/// because its raw count is what the silently-dropped boundary wires reconcile
/// against.
const SURFACE_ENTITIES: [&str; 23] = [
    "ADVANCED_FACE",
    "FACE_SURFACE",
    "PLANE",
    "CYLINDRICAL_SURFACE",
    "CONICAL_SURFACE",
    "SPHERICAL_SURFACE",
    "TOROIDAL_SURFACE",
    "DEGENERATE_TOROIDAL_SURFACE",
    "B_SPLINE_SURFACE_WITH_KNOTS",
    "RATIONAL_B_SPLINE_SURFACE",
    "BEZIER_SURFACE",
    "SURFACE_OF_LINEAR_EXTRUSION",
    "SURFACE_OF_REVOLUTION",
    "MANIFOLD_SOLID_BREP",
    "BREP_WITH_VOIDS",
    "VERTEX_LOOP",
    "ORIENTED_EDGE",
    "ORIENTED_FACE",
    "ORIENTED_OPEN_SHELL",
    "ORIENTED_CLOSED_SHELL",
    "DEFINITIONAL_REPRESENTATION",
    "PRODUCT_DEFINITION_FORMATION",
    "PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE",
];

// ------------------------------------------------------- assembly depth

#[derive(Default)]
struct AssemblyDepth {
    solids: usize,
    /// Outer shell built and kept every face the STEP shell listed.
    full: usize,
    /// Outer shell built but lost at least one face.
    partial: usize,
    /// `to_compressed_shell` returned `Err`, or the outer shell was
    /// unreferenceable.
    failed: usize,
    /// `to_compressed_shell` PANICKED. Uncaught, this aborts the whole load.
    panicked: usize,
    faces_listed: usize,
    faces_kept: usize,
    /// Faces lost where the surface DID convert -- so the loss cannot be
    /// attributed to the two `eprintln!`ed conversion arms.
    faces_lost_unexplained: usize,
    /// `cfs_faces` entries whose id is in neither `oriented_face` nor
    /// `face_surface`. Dropped by `cfs_faces_holder` with no message at all.
    face_refs_unresolvable: usize,
    /// Boundary wires the table lists on kept faces, versus wires that survive.
    /// `face_bound_to_edges` drops a wire on `None` with no message.
    wires_listed: usize,
    wires_kept: usize,
    errors: BTreeMap<String, usize>,
}

/// Resolve a `ManifoldSolidBrep`'s outer shell to the underlying `ShellHolder`
/// plus the shell entity id, following the same two lookups
/// `to_compressed_solid` uses.
fn outer_shell(
    table: &Table,
    solid: &monstertruck_step::load::ManifoldSolidBrepHolder,
) -> Option<monstertruck_step::load::ShellHolder> {
    let PlaceHolder::Ref(Name::Entity(id)) = &solid.outer else {
        return None;
    };
    table.shell.get(id).cloned().or_else(|| {
        let holder = table.oriented_shell.get(id)?;
        let PlaceHolder::Ref(Name::Entity(inner)) = &holder.shell_element else {
            return None;
        };
        table.shell.get(inner).cloned()
    })
}

fn assembly_depth(
    table: &Table,
    converted: &std::collections::HashMap<u64, bool>,
) -> AssemblyDepth {
    let mut depth = AssemblyDepth {
        solids: table.manifold_solid_brep.len(),
        ..Default::default()
    };
    table.manifold_solid_brep.values().for_each(|solid| {
        let Some(shell) = outer_shell(table, solid) else {
            depth.failed += 1;
            *depth
                .errors
                .entry("outer shell unreferenceable".to_owned())
                .or_default() += 1;
            return;
        };
        let listed = shell.cfs_faces.len();
        depth.faces_listed += listed;

        // Which `cfs_faces` entries resolve to a face at all, and for the ones
        // that do, did the surface convert and how many wires were listed?
        let resolved: Vec<Option<(u64, usize)>> = shell
            .cfs_faces
            .iter()
            .map(|face| {
                let PlaceHolder::Ref(Name::Entity(id)) = face else {
                    return None;
                };
                let face_id = table
                    .oriented_face
                    .get(id)
                    .and_then(|of| match &of.face_element {
                        PlaceHolder::Ref(Name::Entity(inner)) => Some(*inner),
                        _ => None,
                    })
                    .or_else(|| table.face_surface.contains_key(id).then_some(*id))?;
                let wires = table.face_surface.get(&face_id)?.bounds.len();
                Some((face_id, wires))
            })
            .collect();
        depth.face_refs_unresolvable += resolved.iter().filter(|r| r.is_none()).count();
        let expected_kept = resolved
            .iter()
            .flatten()
            .filter(|(id, _)| converted.get(id).copied().unwrap_or(false))
            .count();
        depth.wires_listed += resolved
            .iter()
            .flatten()
            .filter(|(id, _)| converted.get(id).copied().unwrap_or(false))
            .map(|(_, wires)| wires)
            .sum::<usize>();

        match catching(|| table.to_compressed_shell(&shell)) {
            Err(panic) => {
                depth.panicked += 1;
                *depth
                    .errors
                    .entry(without_ids(&format!("PANIC: {panic}")))
                    .or_default() += 1;
            }
            Ok(Err(e)) => {
                depth.failed += 1;
                *depth.errors.entry(without_ids(&e.to_string())).or_default() += 1;
            }
            Ok(Ok(cshell)) => {
                let kept = cshell.faces.len();
                depth.faces_kept += kept;
                depth.wires_kept += cshell
                    .faces
                    .iter()
                    .map(|f| f.boundaries.len())
                    .sum::<usize>();
                depth.faces_lost_unexplained += expected_kept.saturating_sub(kept);
                if kept == listed {
                    depth.full += 1;
                } else {
                    depth.partial += 1;
                }
            }
        }
    });
    depth
}

// ------------------------------------------------------------- reporting

fn report(label: &str, bytes: &[u8], table: &Table, refusals: &Refusals, with_assembly: bool) {
    let raw = raw_entity_counts(bytes);
    let census = census_faces(table);

    eprintln!("\n=== {label}  ({} MB)", bytes.len() / 1_048_576);
    let swallowed: usize = refusals.values().map(|r| r.count).sum();
    eprintln!("  entity records SWALLOWED by push_instance (load still returned Ok): {swallowed}");
    refusals.iter().for_each(|((name, error), r)| {
        eprintln!(
            "    {name:<52} x{:<6} e.g. #{:?}  {error}",
            r.count, r.sample_ids,
        );
    });
    eprintln!(
        "  faces: raw ADVANCED_FACE={} + FACE_SURFACE={}  ->  table.face_surface={}",
        raw.get("ADVANCED_FACE").copied().unwrap_or(0),
        raw.get("FACE_SURFACE").copied().unwrap_or(0),
        census.faces_in_table,
    );
    eprintln!("  raw entity counts vs table entries:");
    SURFACE_ENTITIES.iter().for_each(|name| {
        let n = raw.get(*name).copied().unwrap_or(0);
        if n > 0 {
            eprintln!("    {name:<32} raw={n}");
        }
    });

    // Several `push_instance` arms insert only when a `params.len()` guard
    // holds, and when it does NOT hold they fall off the end of the arm --
    // no insert, no `Err`, no message. That kind of loss is invisible to the
    // refusal census above, so it can only be seen by reconciling raw record
    // counts against table entries. A non-zero `MISSING` here is a silent drop.
    eprintln!("  guarded-arm reconciliation (raw records -> table entries):");
    let count_of = |names: &[&str]| {
        names
            .iter()
            .map(|n| raw.get(*n).copied().unwrap_or(0))
            .sum()
    };
    let guarded: [(&str, usize, usize); 6] = [
        (
            "oriented_edge",
            count_of(&["ORIENTED_EDGE"]),
            table.oriented_edge.len(),
        ),
        (
            "oriented_face",
            count_of(&["ORIENTED_FACE"]),
            table.oriented_face.len(),
        ),
        (
            "oriented_shell",
            count_of(&["ORIENTED_OPEN_SHELL", "ORIENTED_CLOSED_SHELL"]),
            table.oriented_shell.len(),
        ),
        (
            "manifold_solid_brep",
            count_of(&["MANIFOLD_SOLID_BREP", "BREP_WITH_VOIDS"]),
            table.manifold_solid_brep.len(),
        ),
        (
            "definitional_representation",
            count_of(&["DEFINITIONAL_REPRESENTATION"]),
            table.definitional_representation.len(),
        ),
        (
            "product_definition_formation",
            count_of(&[
                "PRODUCT_DEFINITION_FORMATION",
                "PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE",
            ]),
            table.product_definition_formation.len(),
        ),
    ];
    guarded.iter().for_each(|(field, raw_n, table_n)| {
        if *raw_n > 0 || *table_n > 0 {
            let missing = raw_n.saturating_sub(*table_n);
            let flag = if missing > 0 { "  <<< MISSING" } else { "" };
            eprintln!("    {field:<32} raw={raw_n:<7} table={table_n:<7} missing={missing}{flag}");
        }
    });

    eprintln!("  per-class face fates:");
    let mut converted_total = 0;
    let mut refused_total = 0;
    let mut panicked_total = 0;
    census.by_class.iter().for_each(|(class, tally)| {
        converted_total += tally.converted;
        refused_total += tally.refused();
        panicked_total += tally.panics();
        let pct = 100.0 * tally.converted as f64 / tally.total() as f64;
        eprintln!(
            "    {:<44} faces={:<7} converted={:<7} ({pct:>5.1}%) refused={:<7} PANICKED={}",
            class.label(),
            tally.total(),
            tally.converted,
            tally.refused(),
            tally.panics(),
        );
        tally.refused_conversion.iter().for_each(|(e, n)| {
            eprintln!("        conversion refused x{n}: {e}");
        });
        tally.refused_resolution.iter().for_each(|(e, n)| {
            eprintln!("        resolution refused x{n}: {e}");
        });
        tally.panicked.iter().for_each(|(e, n)| {
            eprintln!("        PANIC x{n}: {e}");
        });
    });
    eprintln!(
        "  TOTAL faces={} converted={converted_total} ({:.1}%) refused={refused_total} PANICKED={panicked_total}",
        census.faces_in_table,
        100.0 * converted_total as f64 / census.faces_in_table.max(1) as f64,
    );

    // Internal consistency: every face landed in exactly one bucket.
    let bucket_sum: usize = census.by_class.values().map(ClassTally::total).sum();
    assert_eq!(
        bucket_sum, census.faces_in_table,
        "{label}: buckets must partition the table's faces",
    );

    if with_assembly {
        let depth = assembly_depth(table, &census.converted);
        eprintln!(
            "  solids={} -> shells: full={} partial={} failed={} PANICKED={}",
            depth.solids, depth.full, depth.partial, depth.failed, depth.panicked,
        );
        eprintln!(
            "    faces listed by shells={} kept={} lost={} (of which unexplained={})",
            depth.faces_listed,
            depth.faces_kept,
            depth.faces_listed.saturating_sub(depth.faces_kept),
            depth.faces_lost_unexplained,
        );
        eprintln!(
            "    cfs_faces refs that resolve to no face at all (silent)={}",
            depth.face_refs_unresolvable,
        );
        eprintln!(
            "    wires on kept faces: listed={} kept={} lost={}",
            depth.wires_listed,
            depth.wires_kept,
            depth.wires_listed.saturating_sub(depth.wires_kept),
        );
        depth
            .errors
            .iter()
            .for_each(|(e, n)| eprintln!("    shell error x{n}: {e}"));
        assert_eq!(
            depth.full + depth.partial + depth.failed + depth.panicked,
            depth.solids,
            "{label}: every solid must land in exactly one assembly bucket",
        );
    }
}

// ----------------------------------------------------------------- tests

/// The census over the big-assembly corpus plus every in-repo STEP fixture.
#[test]
#[ignore = "conversion census -- ~1 GB of corpus, run explicitly"]
fn corpus_conversion_census() {
    install_census_panic_hook();
    eprintln!("################ in-repo fixtures ################");
    let mut fixtures = repo_fixtures();
    fixtures.sort();
    fixtures.iter().for_each(|path| {
        let name = path
            .file_name()
            .unwrap_or_default()
            .to_string_lossy()
            .into_owned();
        let Ok(bytes) = std::fs::read(path) else {
            eprintln!("=== {name}  ABSENT");
            return;
        };
        match table_with_refusals(&bytes) {
            Ok((table, refusals)) => report(&name, &bytes, &table, &refusals, true),
            Err(e) => eprintln!("=== {name}  TABLE-PARSE FAILED: {e}"),
        }
    });

    eprintln!("\n################ big-assembly corpus ################");
    let Some(root) = corpus_root() else {
        eprintln!("SKIP: corpus absent; run fixtures/step-bigassy/fetch.sh");
        return;
    };
    let assembly = std::env::var_os("MONSTERTRUCK_CENSUS_NO_ASSEMBLY").is_none();
    // The whole corpus does NOT fit in one test: this repo's nextest config is
    // `slow-timeout = { period = "120s", terminate-after = 10 }`, a hard kill at
    // 1200 s, and a full pass measured 1200 s+ under load. So the census is
    // SHARDABLE -- `MONSTERTRUCK_CENSUS_ONLY=Scania` restricts it to matching
    // file names. Without it you get every file and, on the big corpus, a
    // TIMEOUT rather than a result.
    let only = std::env::var("MONSTERTRUCK_CENSUS_ONLY").unwrap_or_default();
    CORPUS_FILES
        .iter()
        .filter(|name| name.contains(&only))
        .for_each(|name| {
            let Ok(bytes) = std::fs::read(root.join(name)) else {
                eprintln!("=== {name}  ABSENT");
                return;
            };
            let started = std::time::Instant::now();
            match table_with_refusals(&bytes) {
                Ok((table, refusals)) => {
                    report(name, &bytes, &table, &refusals, assembly);
                    eprintln!("  elapsed {:.1}s", started.elapsed().as_secs_f64());
                }
                Err(msg) => {
                    let first = msg
                        .lines()
                        .next()
                        .unwrap_or("")
                        .chars()
                        .take(120)
                        .collect::<String>();
                    eprintln!(
                        "\n=== {name}  ({} MB)  TABLE-PARSE FAILED",
                        bytes.len() / 1_048_576
                    );
                    eprintln!("  {first}");
                }
            }
        });
}

/// A minimal DATA section carrying two faces that differ ONLY in the surface
/// subtype: `#5` is a `DEGENERATE_TOROIDAL_SURFACE`, `#15` a plain
/// `TOROIDAL_SURFACE`. Placement, bound, loop, edge and vertex are all shared,
/// so any difference in outcome is attributable to the subtype and nothing else.
///
/// The degenerate numbers are lifted verbatim from `Rocky_House.stp` `#145`
/// (major 0.633974596215563 < minor 1.0 -- the self-intersecting regime AP214
/// splits the entity for).
const TORUS_DIFFERENTIAL: &str = "DATA;
#1=CARTESIAN_POINT('',(0.,0.,0.));
#2=DIRECTION('',(0.,0.,1.));
#3=DIRECTION('',(1.,0.,0.));
#4=AXIS2_PLACEMENT_3D('',#1,#2,#3);
#5=DEGENERATE_TOROIDAL_SURFACE('',#4,0.633974596215563,1.,.T.);
#6=VERTEX_POINT('',#1);
#7=VECTOR('',#2,1.);
#8=LINE('',#1,#7);
#9=EDGE_CURVE('',#6,#6,#8,.T.);
#10=ORIENTED_EDGE('',*,*,#9,.T.);
#11=EDGE_LOOP('',(#10));
#12=FACE_OUTER_BOUND('',#11,.T.);
#13=ADVANCED_FACE('',(#12),#5,.T.);
#15=TOROIDAL_SURFACE('',#4,3.,1.);
#16=ADVANCED_FACE('',(#12),#15,.T.);
ENDSEC;
";

/// **The T1 decision test, INVERTED at the T1 landing.** What happens to a face
/// whose surface is a `DEGENERATE_TOROIDAL_SURFACE`? This pins the answer without
/// needing the corpus.
///
/// Before T1 (measured 2026-07-29): the record fell into `Table::dummy` because
/// `ElementarySurfaceAny` had no variant for the subtype, and resolving the face
/// refused with `Lookup failed for #5` -- typed, but naming a lookup miss rather
/// than the class or the reason.
///
/// After T1 the same face still refuses, and still is never mistaken for a plain
/// torus, but:
///
/// 1. the record deserializes into its OWN table map, name and all five
///    attributes intact -- nothing lands in `dummy`;
/// 2. it is NOT in `toroidal_surface`. If it ever were, the degenerate
///    parameterisation would be silently reinterpreted as a plain torus, which
///    is a silent-WRONG, strictly worse than losing the face;
/// 3. the refusal moved from RESOLUTION to CONVERSION and now names the class,
///    the two radii and the regime. The plain ring torus in the same DATA
///    section still converts, proving the refusal is the geometry's fault and
///    not the fixture's.
#[test]
fn degenerate_toroidal_surface_is_refused_typed_not_mistaken_for_a_torus() {
    let section = DataSection::from_str(TORUS_DIFFERENTIAL).expect("the DATA section must parse");
    let table = Table::from_data_section(&section);

    // 1. The record reached the table as a named class, not as a dummy.
    assert!(
        table.degenerate_toroidal_surface.contains_key(&5),
        "#5 must deserialize into Table::degenerate_toroidal_surface",
    );
    assert!(
        !table.dummy.contains_key(&5),
        "a schema entity must not fall into Table::dummy any more",
    );

    // 2. It was NOT absorbed as a plain torus.
    assert!(
        !table.toroidal_surface.contains_key(&5),
        "a degenerate torus must never land in the plain-torus map -- that would be silent-wrong",
    );

    // 3. The face refuses at CONVERSION, naming the class; the ring twin converts.
    let degenerate = table.face_surface.get(&13).expect("#13 must be a face");
    let plain = table.face_surface.get(&16).expect("#16 must be a face");
    let fate = fate_of(&table, degenerate);
    let Fate::RefusedConversion(message) = &fate else {
        panic!("the degenerate torus face must refuse at conversion, got {fate:?}");
    };
    assert!(
        message.starts_with("DEGENERATE_TOROIDAL_SURFACE refused: "),
        "the refusal must name the class, not a lookup miss: {message}",
    );
    assert!(
        message.contains("major_radius 0.633974596215563")
            && message.contains("minor_radius 1")
            && message.contains("self-intersecting"),
        "the refusal must name the radii and the regime: {message}",
    );
    assert_eq!(
        fate_of(&table, plain),
        Fate::Converted,
        "the plain torus face in the SAME data section must convert",
    );
}

/// A `DEGENERATE_TOROIDAL_SURFACE` whose radii are an EXACT HORN torus
/// (`3., 3.`), copied verbatim from `Ai-14R.stp` -- one of the 253 records is
/// spelled degenerate but is not in the degenerate regime at all (it violates the
/// subtype's own `major_radius < minor_radius` rule). A horn torus is
/// representable: the inner equator pinches to a point on the axis, nothing
/// self-intersects, and parameter recovery measures 576/576 exact. So the RADII
/// decide the fate, not the spelling -- and `select_outer` decides the sheet:
/// `.T.` is the whole surface, `.F.` is the collapsed apex, which is not one.
const HORN_AS_DEGENERATE_TORUS: &str = "DATA;
#1=CARTESIAN_POINT('',(0.,0.,0.));
#2=DIRECTION('',(0.,0.,1.));
#3=DIRECTION('',(1.,0.,0.));
#4=AXIS2_PLACEMENT_3D('',#1,#2,#3);
#5=DEGENERATE_TOROIDAL_SURFACE('',#4,3.,3.,.T.);
#6=VERTEX_POINT('',#1);
#7=VECTOR('',#2,1.);
#8=LINE('',#1,#7);
#9=EDGE_CURVE('',#6,#6,#8,.T.);
#10=ORIENTED_EDGE('',*,*,#9,.T.);
#11=EDGE_LOOP('',(#10));
#12=FACE_OUTER_BOUND('',#11,.T.);
#13=ADVANCED_FACE('',(#12),#5,.T.);
#15=DEGENERATE_TOROIDAL_SURFACE('',#4,3.,3.,.F.);
#16=ADVANCED_FACE('',(#12),#15,.T.);
ENDSEC;
";

#[test]
fn a_horn_torus_spelled_degenerate_converts_unless_it_selects_the_apex_sheet() {
    let section =
        DataSection::from_str(HORN_AS_DEGENERATE_TORUS).expect("the DATA section must parse");
    let table = Table::from_data_section(&section);

    let outer = table.face_surface.get(&13).expect("#13 must be a face");
    assert_eq!(
        fate_of(&table, outer),
        Fate::Converted,
        "a horn torus is representable, so select_outer = .T. must convert",
    );

    let inner = table.face_surface.get(&16).expect("#16 must be a face");
    let fate = fate_of(&table, inner);
    let Fate::RefusedConversion(message) = &fate else {
        panic!("select_outer = .F. must refuse at conversion, got {fate:?}");
    };
    assert!(
        message.contains("select_outer = .F.") && message.contains("apex"),
        "the refusal must name select_outer and the apex sheet: {message}",
    );
}

/// The four `$`-bearing assembly records that used to take the whole assembly
/// graph down, copied VERBATIM from `Scania-8x4.stp` with the referenced ids
/// rewritten. Note `ITEM_DEFINED_TRANSFORMATION($,$,..)` and
/// `PRODUCT_DEFINITION_SHAPE($,$,..)`: it is not only `description` that is
/// unset, it is `name` as well -- the reason a strictly schema-faithful
/// description-only fix would still have lost 470 of Scania-8x4's 695
/// `PRODUCT_DEFINITION_SHAPE` records and all 470 of its transformations.
///
/// `#7` is the deliberate control: `ProductDefinition.id` is `identifier`,
/// mandatory, unexercised by any measured file, and therefore still `String`.
/// It must STILL refuse -- and now be counted.
const NULL_LABEL_ASSEMBLY: &str = "DATA;
#1=PRODUCT_DEFINITION_SHAPE('',$,#2);
#2=PRODUCT_DEFINITION('','',#3,#4);
#3=PRODUCT_DEFINITION_SHAPE($,$,#2);
#4=PRODUCT_DEFINITION_FORMATION('',$,#5);
#5=PRODUCT('name','name',$,(#6));
#6=ITEM_DEFINED_TRANSFORMATION($,$,#8,#9);
#7=PRODUCT_DEFINITION($,'',#3,#4);
ENDSEC;
";

/// **The swallowed-entity pin, INVERTED at the T6 landing.**
///
/// Before T6 (measured 2026-07-29): `Table::from_iter` did
/// `push_instance(..).unwrap_or_else(|e| eprintln!("{e}"))`, so a record whose
/// `Deserialize` refused was printed to stderr and then DROPPED. The entity
/// simply did not exist in the resulting table, the load still returned `Ok`, and
/// there was no third state to inspect. 4,562 corpus records went that way, being
/// 100% of four entity types on both Scania files.
///
/// After T6, three things must hold at once, and the test checks all three
/// because any one of them alone would be a false comfort:
///
/// 1. the `$`-bearing records DESERIALIZE, with `None` where the file wrote `$`
///    and `Some("")` where it wrote `''` -- the distinction survives;
/// 2. a record that still refuses is still refused (nothing was made lenient
///    wholesale) AND is now COUNTED in `Table::entity_report`, per entity type,
///    with an example id;
/// 3. the load still returns `Ok`. Loss is VISIBLE, not fatal.
#[test]
fn swallowed_entity_records_are_counted_and_the_null_label_family_now_loads() {
    let section = DataSection::from_str(NULL_LABEL_ASSEMBLY).expect("the DATA section must parse");
    let table = Table::from_data_section(&section);

    // 1. The `$`-bearing records reached the table, and `$` != `''`.
    let pds = table
        .product_definition_shape
        .get(&1)
        .expect("#1 PRODUCT_DEFINITION_SHAPE('',$,..) must deserialize");
    assert_eq!(pds.name.as_deref(), Some(""), "'' must stay Some(\"\")");
    assert_eq!(pds.description, None, "$ must become None");
    assert!(
        table.product_definition_shape.contains_key(&3),
        "#3 PRODUCT_DEFINITION_SHAPE($,$,..) -- name unset too -- must deserialize",
    );
    assert_eq!(
        table.product_definition_shape.len(),
        2,
        "both PRODUCT_DEFINITION_SHAPE records must be present",
    );
    assert!(
        table.product_definition_formation.contains_key(&4)
            && table.product.contains_key(&5)
            && table.item_defined_transformation.contains_key(&6),
        "the other three members of the assembly quartet must deserialize too",
    );

    // 2. The control still refuses -- and the refusal is now attributable.
    assert!(
        !table.product_definition.contains_key(&7),
        "#7 PRODUCT_DEFINITION($,..) must STILL refuse: `id` is mandatory `identifier` \
         and no measured file writes `$` there, so it was deliberately left `String`",
    );
    let report = &table.entity_report;
    assert_eq!(
        report.total(),
        1,
        "exactly one record was swallowed: {report}"
    );
    assert_eq!(report.swallowed_of("PRODUCT_DEFINITION"), 1);
    assert_eq!(report.swallowed_of("PRODUCT_DEFINITION_SHAPE"), 0);
    let (name, tally) = report
        .refused()
        .next()
        .expect("the swallow must be tallied under its entity type name");
    assert_eq!(name, "PRODUCT_DEFINITION");
    assert_eq!(
        tally.first_id,
        Some(7),
        "the tally must name a record to go and look at"
    );
    assert_eq!(
        tally.first_detail.as_deref(),
        Some("Error while deserialize STEP struct: invalid type: Option value, expected a string"),
        "the refusal text is the one the corpus load emitted by the thousand",
    );

    // 3. `push_instance` itself still returns the typed error, and the aggregate
    //    route still returns a Table -- loss is visible, not fatal.
    let mut probe = Table::default();
    probe
        .push_instance(&section.entities[6])
        .expect_err("a null mandatory identifier must still refuse");
    assert!(
        report.require_empty().is_err(),
        "a caller that WANTS strictness must be able to get it in one line",
    );
}

/// A plain `TOROIDAL_SURFACE` whose major radius is NEGATIVE, copied verbatim
/// from `ROTOR-201NAL-Z7.STEP` `#53555` (`-53.10833468790826100, 66.0`). Note
/// `|major| < minor`: this is the SAME degenerate/self-intersecting regime as
/// `DEGENERATE_TOROIDAL_SURFACE`, just spelled by a different exporter.
const NEGATIVE_MAJOR_RADIUS_TORUS: &str = "DATA;
#1=CARTESIAN_POINT('',(0.,0.,0.));
#2=DIRECTION('',(0.,0.,1.));
#3=DIRECTION('',(1.,0.,0.));
#4=AXIS2_PLACEMENT_3D('',#1,#2,#3);
#5=TOROIDAL_SURFACE('',#4,-53.10833468790826100,66.00000000000000000);
#6=VERTEX_POINT('',#1);
#7=VECTOR('',#2,1.);
#8=LINE('',#1,#7);
#9=EDGE_CURVE('',#6,#6,#8,.T.);
#10=ORIENTED_EDGE('',*,*,#9,.T.);
#11=EDGE_LOOP('',(#10));
#12=FACE_OUTER_BOUND('',#11,.T.);
#13=ADVANCED_FACE('',(#12),#5,.T.);
ENDSEC;
";

/// **The inverted STOP-class pin.** Measured 2026-07-29, a negative-major-radius
/// `TOROIDAL_SURFACE` -- 201 in `NissanGT-R.STEP`, 6 in `ROTOR-201NAL-Z7.STEP` --
/// did not refuse: it PANICKED in `Torus::new`
/// (`monstertruck-geometry/src/specifieds/torus.rs:9`), because the STEP
/// `From<&ToroidalSurface>` impl was INFALLIBLE and handed the radii straight to a
/// constructor that panics on `large_radius <= 0.0`. Uncaught, that aborted the
/// load of eight NissanGT-R solids -- 15,109 of 41,071 shell faces -- while
/// `corpus_load` still called the file OK.
///
/// Spec 011 T1 made the conversion fallible (`TryFrom<&ToroidalSurface>`). This
/// test is the inversion the Phase 0 census demanded: the SAME record must now
/// refuse typed, name the class and never unwind. `catching` is what proves the
/// no-panic half -- it would report the unwind rather than let it escape.
#[test]
fn negative_major_radius_torus_refuses_typed_and_does_not_panic() {
    let section =
        DataSection::from_str(NEGATIVE_MAJOR_RADIUS_TORUS).expect("the DATA section must parse");
    let table = Table::from_data_section(&section);
    let face = table.face_surface.get(&13).expect("#13 must be a face");
    let surface_any = face
        .face_geometry
        .clone()
        .into_owned(&table)
        .expect("the torus record must resolve");

    let converted = catching(|| Surface::try_from(&surface_any))
        .expect("conversion must not panic on a negative major radius");
    let error = converted
        .expect_err("a negative major radius must refuse")
        .to_string();
    assert!(
        error.starts_with("TOROIDAL_SURFACE refused: non-positive major_radius"),
        "the refusal must name the class and the offending attribute: {error}",
    );
    assert!(
        error.contains("-53.10833468790826") && error.contains("minor_radius 66"),
        "the refusal must quote both radii: {error}",
    );

    // And the whole-face route agrees, so the census sees a refusal, not a panic.
    assert!(
        matches!(fate_of(&table, face), Fate::RefusedConversion(_)),
        "the face fate must be a typed conversion refusal",
    );
}

/// A plain `TOROIDAL_SURFACE` with a POSITIVE major radius that is nonetheless
/// SMALLER than the minor radius -- `#217930` of `NissanGT-R.STEP`
/// (`37.525205668822764, 94.`). This is the THIRD spelling of the degenerate
/// regime, and the Phase 0 census could not see it: such a record passes
/// `Torus::new` (which only rejects radii `<= 0`), so it counted as `converted`.
/// Measured 2026-07-30: 136 corpus records (NissanGT-R 126, UMC-500 10) are in
/// this state.
///
/// It was never representable -- the rational-NURBS builder returns `None` for it
/// and `Torus::search_nearest_parameter` answers 25-29% of its domain with
/// parameters that evaluate to a different point -- so it now refuses typed at the
/// same place as the other two spellings, with the same message modulo the entity
/// name.
const POSITIVE_MAJOR_SPINDLE_TORUS: &str = "DATA;
#1=CARTESIAN_POINT('',(0.,0.,0.));
#2=DIRECTION('',(0.,0.,1.));
#3=DIRECTION('',(1.,0.,0.));
#4=AXIS2_PLACEMENT_3D('',#1,#2,#3);
#5=TOROIDAL_SURFACE('',#4,37.525205668822764,94.);
#6=VERTEX_POINT('',#1);
#7=VECTOR('',#2,1.);
#8=LINE('',#1,#7);
#9=EDGE_CURVE('',#6,#6,#8,.T.);
#10=ORIENTED_EDGE('',*,*,#9,.T.);
#11=EDGE_LOOP('',(#10));
#12=FACE_OUTER_BOUND('',#11,.T.);
#13=ADVANCED_FACE('',(#12),#5,.T.);
ENDSEC;
";

#[test]
fn positive_major_spindle_torus_refuses_typed_naming_the_regime() {
    let section =
        DataSection::from_str(POSITIVE_MAJOR_SPINDLE_TORUS).expect("the DATA section must parse");
    let table = Table::from_data_section(&section);
    let face = table.face_surface.get(&13).expect("#13 must be a face");
    let fate = fate_of(&table, face);
    let Fate::RefusedConversion(message) = &fate else {
        panic!("a positive-major spindle torus must refuse at conversion, got {fate:?}");
    };
    assert!(
        message.starts_with("TOROIDAL_SURFACE refused: degenerate self-intersecting torus"),
        "the refusal must name the regime: {message}",
    );
    assert!(
        message.contains("major_radius 37.525205668822764") && message.contains("minor_radius 94"),
        "the refusal must quote both radii: {message}",
    );
}

/// The control group for all three refusals: a horn torus (`major == minor`) and
/// the fp-near-horn form real STEP fillets take (the two radii differing by a few
/// ulps) must KEEP converting. The refusal predicate is deliberately the geometry
/// builder's own, so it must not widen by so much as an ulp -- four of
/// `raspberry-pi-4-model-b.step`'s ten tori are near-horn and convert in the
/// default gate today.
#[test]
fn horn_and_near_horn_tori_still_convert() {
    for (label, radii) in [
        ("exact horn", "3.,3."),
        (
            "fp near-horn (Pi fillet)",
            "0.099999999992725,0.09999999999998788",
        ),
        ("ring", "15.,9."),
    ] {
        let step = format!(
            "DATA;
#1=CARTESIAN_POINT('',(0.,0.,0.));
#2=DIRECTION('',(0.,0.,1.));
#3=DIRECTION('',(1.,0.,0.));
#4=AXIS2_PLACEMENT_3D('',#1,#2,#3);
#5=TOROIDAL_SURFACE('',#4,{radii});
#6=VERTEX_POINT('',#1);
#7=VECTOR('',#2,1.);
#8=LINE('',#1,#7);
#9=EDGE_CURVE('',#6,#6,#8,.T.);
#10=ORIENTED_EDGE('',*,*,#9,.T.);
#11=EDGE_LOOP('',(#10));
#12=FACE_OUTER_BOUND('',#11,.T.);
#13=ADVANCED_FACE('',(#12),#5,.T.);
ENDSEC;
"
        );
        let section = DataSection::from_str(&step).expect("the DATA section must parse");
        let table = Table::from_data_section(&section);
        let face = table.face_surface.get(&13).expect("#13 must be a face");
        assert_eq!(
            fate_of(&table, face),
            Fate::Converted,
            "the {label} torus ({radii}) must still convert",
        );
    }
}

/// The raw counter must see both STEP formatting styles. The compact form is
/// what most exporters emit; the spaced form is what three of the eight corpus
/// files emit, and a counter that misses it reports a confident zero for a file
/// full of geometry. Also pins that a longer entity name is not double-counted
/// as its own suffix (`DEGENERATE_TOROIDAL_SURFACE` vs `TOROIDAL_SURFACE`).
#[test]
fn raw_entity_counter_sees_both_step_formatting_styles() {
    let compact = b"#145=DEGENERATE_TOROIDAL_SURFACE('',#1,0.6,1.,.T.);\n#146=TOROIDAL_SURFACE('',#2,3.,1.);\n";
    let spaced = b"#5 = ADVANCED_FACE ( 'NONE' , ( #6 ) , #7 , .F. ) ;\r\n#8 = ADVANCED_FACE ( '' , ( ) , #9 , .T. ) ;\r\n";

    let c = raw_entity_counts(compact);
    assert_eq!(c.get("DEGENERATE_TOROIDAL_SURFACE").copied(), Some(1));
    assert_eq!(
        c.get("TOROIDAL_SURFACE").copied(),
        Some(1),
        "the degenerate name must not also count as a plain torus",
    );

    let s = raw_entity_counts(spaced);
    assert_eq!(
        s.get("ADVANCED_FACE").copied(),
        Some(2),
        "space-separated records must be counted; three corpus files use this form",
    );
}

/// The bucket logic itself, on a fixture that is always present and cheap. This
/// is the part of the census that must not rot: if `Fate` ever stopped
/// partitioning the faces, every number above would be quietly wrong.
#[test]
fn face_fates_partition_a_small_fixture() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../resources/step/occt-torus.step");
    let bytes = std::fs::read(&path).expect("occt-torus.step must be present");
    let table = Table::from_step_bytes(&bytes).expect("occt-torus.step must table-parse");
    let census = census_faces(&table);
    assert!(
        census.faces_in_table > 0,
        "the torus fixture must contain faces"
    );
    assert_eq!(
        census
            .by_class
            .values()
            .map(ClassTally::total)
            .sum::<usize>(),
        census.faces_in_table,
        "the buckets must partition the faces",
    );
    assert_eq!(
        census.converted.len(),
        census.faces_in_table,
        "every face must get exactly one memoised fate",
    );
}
