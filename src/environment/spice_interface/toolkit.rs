//! The process-wide CSPICE toolkit, behind one lock, and its kernel cache
//! (upstream `SpiceKernel` and `SpiceErrorModeGuard`).
//!
//! CSPICE is one global state and is not thread-safe, so every call into it
//! goes through a [`Toolkit`] borrowed from [`lock`]. The toolkit runs in the
//! `RETURN` error action with the error device off: a failing routine sets
//! the error state instead of aborting the process, and each wrapper here
//! turns that state into an `Err` carrying CSPICE's message and clears it.
//! Upstream switches to `RETURN` only around kernel loads and knot queries
//! (`SpiceErrorModeGuard`) and prints the messages.

use std::collections::HashMap;
use std::path::{Component, Path, PathBuf};
use std::sync::{Arc, LazyLock, Mutex, MutexGuard, PoisonError, Weak};

use nalgebra::{Matrix3, Vector3};

/// The toolkit and the kernels it holds, keyed by canonical path; a live
/// [`Weak`] means a [`SpiceKernel`] still owns that file.
pub(crate) struct Toolkit {
    kernels: HashMap<String, Weak<SpiceKernel>>,
}

static TOOLKIT: LazyLock<Mutex<Toolkit>> = LazyLock::new(|| {
    spice::errors::erract("SET", "RETURN");
    spice::errors::errdev("SET", "NULL");
    Mutex::new(Toolkit {
        kernels: HashMap::new(),
    })
});

/// The toolkit, locked for the caller's exclusive use. A panic while it was
/// held leaves no CSPICE error pending (each wrapper clears it), so a
/// poisoned lock is taken over.
pub(crate) fn lock() -> MutexGuard<'static, Toolkit> {
    TOOLKIT.lock().unwrap_or_else(PoisonError::into_inner)
}

/// CSPICE's error after a call, cleared, as `Err`.
fn checked<T>(value: T) -> Result<T, String> {
    spice::errors::check()
        .map(|()| value)
        .map_err(|error| error.to_string())
}

impl Toolkit {
    /// `str2et_c`: ephemeris seconds past J2000 of a time string.
    pub(crate) fn str2et(&mut self, time: &str) -> Result<f64, String> {
        checked(spice::str2et(time))
    }

    /// `unitim_c`: `epoch` converted from time system `from` to `to`.
    pub(crate) fn unitim(&mut self, epoch: f64, from: &str, to: &str) -> Result<f64, String> {
        checked(spice::unitim(epoch, from, to))
    }

    /// `deltet_c`: ET - UTC \[s\] at `epoch` of kind `"ET"` or `"UTC"`.
    pub(crate) fn deltet(&mut self, epoch: f64, kind: &str) -> Result<f64, String> {
        checked(spice::deltet(epoch, kind))
    }

    /// `timout_c` with output capacity `capacity`, terminator included.
    pub(crate) fn timout(
        &mut self,
        et: f64,
        picture: &str,
        capacity: i32,
    ) -> Result<String, String> {
        checked(spice::raw::timout(et, picture, capacity))
    }

    /// `et2utc_c`.
    #[cfg(test)]
    pub(crate) fn et2utc(
        &mut self,
        et: f64,
        format: &str,
        precision: i32,
    ) -> Result<String, String> {
        checked(spice::et2utc(et, format, precision))
    }

    /// `cnmfrm_c`: whether the object `name` has an associated frame.
    pub(crate) fn has_frame(&mut self, name: &str) -> Result<bool, String> {
        let (_, _, found) = spice::cnmfrm(name, CHAR_BUFFER_SIZE);
        checked(found)
    }

    /// `spkezr_c` without aberration correction, as position \[m\] and
    /// velocity \[m/s\].
    pub(crate) fn state(
        &mut self,
        target: &str,
        et: f64,
        frame: &str,
        observer: &str,
    ) -> Result<(Vector3<f64>, Vector3<f64>), String> {
        let (state_km, _light_time) = checked(spice::spkezr(target, et, frame, "NONE", observer))?;
        Ok((
            Vector3::new(state_km[0], state_km[1], state_km[2]) * 1000.0,
            Vector3::new(state_km[3], state_km[4], state_km[5]) * 1000.0,
        ))
    }

    /// `sxform_c`: the DCM from `from` to `to` and its rate, the upper-left
    /// and lower-left blocks of the state transformation (upstream
    /// `m66Get33Matrix(0, 0)` and `(1, 0)`).
    pub(crate) fn orientation(
        &mut self,
        from: &str,
        to: &str,
        et: f64,
    ) -> Result<(Matrix3<f64>, Matrix3<f64>), String> {
        let transform = checked(spice::sxform(from, to, et))?;
        let block = |row: usize| Matrix3::from_fn(|r, c| transform[row + r][c]);
        Ok((block(0), block(3)))
    }

    /// `furnsh_c`.
    fn furnish(&mut self, path: &str) -> Result<(), String> {
        spice::furnsh(path);
        checked(())
    }

    /// `unload_c`.
    fn unload(&mut self, path: &str) -> Result<(), String> {
        spice::unload(path);
        checked(())
    }

    /// `ktotal_c("ALL")`: the number of loaded kernels.
    pub(crate) fn kernel_count(&mut self) -> usize {
        usize::try_from(spice::ktotal("ALL")).unwrap_or(0)
    }

    /// Whether the kernel file `path` is loaded (`kdata_c` over all kernels).
    pub(crate) fn is_loaded(&mut self, path: &str) -> bool {
        (0..spice::ktotal("ALL")).any(|which| {
            let (file, _, _, _, found) = spice::kdata(which, "ALL");
            found && file == path
        })
    }
}

/// Upstream `charBufferSize`: capacity of the frame-name buffer, terminator
/// included.
const CHAR_BUFFER_SIZE: i32 = 512;

/// One furnished kernel (upstream `SpiceKernel`): loaded once per canonical
/// path however many interfaces request it, and unloaded when the last
/// handle drops.
#[derive(Debug)]
pub(crate) struct SpiceKernel {
    path: String,
}

impl SpiceKernel {
    /// Upstream `SpiceKernel::request`: a handle to the kernel at `path`,
    /// furnishing it unless a live handle already holds it. A relative path
    /// is taken from the repository root, as the ANISE ephemeris models do.
    pub(crate) fn request(path: &Path) -> Result<Arc<Self>, String> {
        let key = canonical_path(path);
        let mut toolkit = lock();
        if let Some(existing) = toolkit.kernels.get(&key).and_then(Weak::upgrade) {
            return Ok(existing);
        }
        toolkit.furnish(&key)?;
        let kernel = Arc::new(Self { path: key.clone() });
        toolkit.kernels.insert(key, Arc::downgrade(&kernel));
        Ok(kernel)
    }

    /// Upstream `getPath`: the canonical path, the cache key.
    pub(crate) fn path(&self) -> &str {
        &self.path
    }
}

impl Drop for SpiceKernel {
    fn drop(&mut self) {
        let mut toolkit = lock();
        // A request racing this drop finds the expired entry and furnishes the
        // file again under a new handle, which then owns it.
        if toolkit
            .kernels
            .get(&self.path)
            .is_some_and(|entry| entry.strong_count() > 0)
        {
            return;
        }
        toolkit.kernels.remove(&self.path);
        if let Err(message) = toolkit.unload(&self.path) {
            log::warn!("spiceInterface: unloading {} failed: {message}", self.path);
        }
    }
}

/// Upstream `absolutize`: the absolute, lexically normalized form of `path`,
/// a relative path taken from the repository root.
pub(crate) fn canonical_path(path: &Path) -> String {
    let absolute = if path.is_absolute() {
        path.to_path_buf()
    } else {
        Path::new(env!("CARGO_MANIFEST_DIR")).join(path)
    };
    let mut normal = PathBuf::new();
    for component in absolute.components() {
        match component {
            Component::CurDir => {}
            Component::ParentDir => {
                normal.pop();
            }
            other => normal.push(other),
        }
    }
    normal.to_string_lossy().into_owned()
}
