# IGES fixtures: where they came from, and what they can be used for

Both files are stored in Git LFS (see `/.gitattributes`). A checkout without LFS
objects gives you a ~130-byte pointer where the fixture should be;
`the_fixtures_are_real_files_and_not_lfs_pointers` in
`monstertruck-io/tests/iges_fixtures.rs` catches that and says so.

## `nist-ex1.iges` -- 6 KB, IGES 4.0

The worked example from **Appendix A of the IGES specification**, an integrated
circuit semicustom cell. Its own Global section credits the *IGES RFC Review
Committee* at *IPO/NIST*, and its Start section says it comes from IGES version
3.0 modified for 4.0.

A US federal standards-body example. Safe to redistribute, including publicly.

## `robot-rodney.igs` -- 28 MB, IGES 5.2

Downloaded from **3dexport.com** (`3dexport_robot_rodney_1767450377.zip`, model
files dated 2021-09-19). A commercial marketplace model.

**Do not copy this file into the public repository.** No licence accompanied the
download, and 3dexport's usual terms are royalty-free *use*, not redistribution.
It is here as an internal test corpus, which is a different thing from shipping
it. The public `monstertruck` repository publishes to crates.io, so anything
placed there is redistributed to everyone who downloads the crate.

## What they currently prove

Nothing about conversion, yet. **`cadmpeg-codec-iges` 0.4 decodes IGES 5.3 only**
and refuses both of these on version: 4.0 and 5.2 respectively. So the wall these
fixtures hit is in the dependency, not in `src/cadmpeg/convert/` -- no work on the
converter moves them.

They are kept because that is worth pinning. When cadmpeg gains 4.0/5.2 decoding,
`the_iges_fixtures_are_refused_by_the_decoders_version_ceiling` fails on purpose
and tells whoever sees it to write the real end-to-end assertions.

## Still wanted: a 5.3 fixture

None of the files to hand are 5.3, so the converter has still never seen bytes
from a file. A small 5.3 export -- a single trimmed plane is enough -- would close
spec 028 slice 3 properly.

## Not included

`3dexport_speedwayigs_1632303877.rar` held `speedWay.igs`, 91 MB. It was not
added: too large to be a useful fixture, the same redistribution question as
Rodney, and it could not be extracted here anyway -- the archive is RAR5 with a
compression method `7z` does not support, and no `unar`/`unrar` is installed.
