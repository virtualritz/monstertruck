//! Reading IGES 5.3, via `cadmpeg-codec-iges`.
//!
//! monstertruck has no IGES reader of its own and writing one is not worth the
//! effort: cadmpeg already decodes the format to exact analytic carriers and
//! reports what it could not. This module is only the decoder call; the work is
//! in [`crate::cadmpeg`].

use crate::{
    Result,
    cadmpeg::{ImportedSolid, to_solids},
};
use cadmpeg_codec_core::ReadSeek;
use cadmpeg_ir::codec::{CodecEntry, DecodeOptions};

const FORMAT: &str = "IGES";

/// Read solids from an IGES 5.3 stream.
pub fn from_reader(reader: &mut dyn ReadSeek) -> Result<Vec<ImportedSolid>> {
    let decoded = cadmpeg_codec_iges::IgesCodec
        .decode(reader, &DecodeOptions::default())
        .map_err(|error| crate::Error::Decode {
            format: FORMAT,
            message: error.to_string(),
        })?;
    to_solids(&decoded.ir, FORMAT)
}

/// Read solids from an IGES 5.3 file.
pub fn from_path(path: impl AsRef<std::path::Path>) -> Result<Vec<ImportedSolid>> {
    let mut file = std::fs::File::open(path)?;
    from_reader(&mut file)
}
