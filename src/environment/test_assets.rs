//! Upstream `supportData` files the environment tests read, fetched into the
//! git-ignored `assets/` directory by `just fetch-environment-data`.

use std::path::{Path, PathBuf};

/// The asset at `assets/<relative>`, or `None` when it has not been fetched;
/// the test then prints why it is skipped and returns early.
pub(crate) fn asset_or_skip(relative: &str) -> Option<PathBuf> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("assets")
        .join(relative);
    if path.is_file() {
        Some(path)
    } else {
        eprintln!(
            "skipping: {} is missing; run `just fetch-environment-data` to fetch it",
            path.display()
        );
        None
    }
}

mod tests {
    use super::asset_or_skip;

    #[test]
    fn a_missing_asset_is_skipped() {
        assert_eq!(asset_or_skip("no_such_directory/no_such_file.txt"), None);
    }
}
