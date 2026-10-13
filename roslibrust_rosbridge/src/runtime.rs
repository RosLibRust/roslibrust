//! Async runtime primitives whose implementation depends on the target platform.
//!
//! Native targets use tokio. On wasm32 there is no tokio runtime: tasks run on the browser's
//! event loop and timers use gloo-timers (see `wasm.rs`). Names and signatures mirror tokio's,
//! so call sites are the same for every platform.

/// Macro to spawn a detached task.
///
/// The implementation is chosen at compile time:
/// - Native: `tokio::spawn`, so the future and its output must be `Send + 'static`, and it must
///   be called from within a tokio runtime (it panics otherwise).
/// - wasm32: `wasm_bindgen_futures::spawn_local`, which runs it on the browser's event loop.
///   Only `'static` is required here, but code shared with native must meet the native bounds.
macro_rules! spawn {
    ($future:expr) => {{
        let future = $future;

        #[cfg(not(target_arch = "wasm32"))]
        ::tokio::spawn(future);

        #[cfg(target_arch = "wasm32")]
        ::wasm_bindgen_futures::spawn_local(async move {
            let _ = future.await;
        });
    }};
}
pub(crate) use spawn;

#[cfg(not(target_arch = "wasm32"))]
pub(crate) use tokio::{
    task::spawn_blocking,
    time::{sleep, timeout},
};

#[cfg(target_arch = "wasm32")]
pub(crate) use crate::wasm::{sleep, spawn_blocking, timeout};
