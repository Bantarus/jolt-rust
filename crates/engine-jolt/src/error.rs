//! engine-jolt error types.
//!
//! Errors come from two places: (a) JoltC builders (shape construction
//! fails when settings produce a degenerate shape -- e.g. convex hull
//! collapse, mesh self-intersection); (b) the engine-jolt layer
//! detecting misuse (e.g. accessing a freed body, calling `drive_step`
//! after `world.shutdown()`).
//!
//! All errors are non-allocating except the `Build` variant which
//! carries the JoltC error string verbatim so the consumer can log it.

use thiserror::Error;

#[derive(Debug, Error)]
pub enum JoltError {
    /// Shape construction failed inside JoltC. The string is the
    /// verbatim JoltC error message (e.g. "Convex hull is degenerate").
    #[error("jolt: failed to build {kind}: {message}")]
    Build {
        kind: &'static str,
        message: String,
    },

    /// A body handle referenced an id that the world doesn't recognize
    /// (already removed, or never spawned). Returned by methods like
    /// `BodyInterface::get_position` when the body has been destroyed.
    #[error("jolt: body {0:?} not found in world")]
    UnknownBody(crate::body::BodyId),
}

/// Internal helper: turn JoltC's `*mut JPC_String` error pointer into
/// a `JoltError::Build` after a failing builder returns false.
///
/// Safety: caller guarantees `ptr` is either null OR a valid pointer
/// returned by a `JPC_*Settings_Create` builder. This function reads
/// the C string, copies it into owned memory, and intentionally does
/// NOT free `ptr` because JoltC's lifetime expectations for the error
/// string are not documented; the leak is per-call and bounded.
#[cfg(feature = "native")]
pub(crate) unsafe fn build_error(
    kind: &'static str,
    ptr: *mut joltc_sys::JPC_String,
) -> JoltError {
    if ptr.is_null() {
        return JoltError::Build {
            kind,
            message: "(no error message)".to_string(),
        };
    }
    let c_str = std::ffi::CStr::from_ptr(joltc_sys::JPC_String_c_str(ptr));
    let message = c_str.to_string_lossy().into_owned();
    JoltError::Build { kind, message }
}
