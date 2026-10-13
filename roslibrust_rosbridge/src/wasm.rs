//! Browser-specific code, compiled only for wasm32: the WebSocket (gloo-net) and timers
//! (gloo-timers).
//!
//! roslibrust_common's traits require `Send` futures, but gloo's types hold JS values, which are
//! `!Send` so they are wrapped in `SendWrapper`.

use futures::future::{select, Either};
use futures::{Sink, Stream};
use gloo_net::websocket::{futures::WebSocket, Message, State, WebSocketError};
use gloo_timers::future::TimeoutFuture;
use send_wrapper::SendWrapper;
use std::convert::Infallible;
use std::future::{poll_fn, Future};
use std::pin::{pin, Pin};
use std::task::{Context, Poll};
use std::time::Duration;

/// gloo-net's WebSocket, made `Send` with `SendWrapper`.
/// `SendWrapper` forwards `Stream` but not `Sink`, so this newtype forwards both.
pub(crate) struct BrowserSocket(SendWrapper<WebSocket>);

impl Stream for BrowserSocket {
    type Item = Result<Message, WebSocketError>;

    fn poll_next(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        Pin::new(&mut *self.0).poll_next(cx)
    }
}

impl Sink<Message> for BrowserSocket {
    type Error = WebSocketError;

    fn poll_ready(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Result<(), Self::Error>> {
        Pin::new(&mut *self.0).poll_ready(cx)
    }

    fn start_send(mut self: Pin<&mut Self>, item: Message) -> Result<(), Self::Error> {
        Pin::new(&mut *self.0).start_send(item)
    }

    fn poll_flush(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Result<(), Self::Error>> {
        Pin::new(&mut *self.0).poll_flush(cx)
    }

    fn poll_close(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Result<(), Self::Error>> {
        Pin::new(&mut *self.0).poll_close(cx)
    }
}

/// Opens a browser WebSocket and waits until it is connected, like tokio-tungstenite's
/// `connect_async`. gloo-net's `WebSocket::open` returns before the handshake has completed.
pub(crate) async fn connect(
    url: &str,
) -> Result<BrowserSocket, Box<dyn std::error::Error + Send + Sync>> {
    // Fails only for an invalid URL or a blocked port; connection errors show up below
    let mut socket = BrowserSocket(SendWrapper::new(WebSocket::open(url)?));
    // Pending while the socket is connecting; gloo-net's `open` and `error` handlers wake it
    poll_fn(|cx| Pin::new(&mut socket).poll_ready(cx)).await?;
    match socket.0.state() {
        State::Open => Ok(socket),
        // The connection failed, e.g. nothing is listening at `url`
        _ => Err(WebSocketError::ConnectionError.into()),
    }
}

/// Mirrors `tokio::time::sleep`.
pub(crate) fn sleep(duration: Duration) -> SendWrapper<TimeoutFuture> {
    // setTimeout takes an i32 delay, and gloo-timers makes longer delays fire immediately.
    // Clamp them to the longest delay browsers support (about 24.8 days).
    let millis = duration.as_millis().min(i32::MAX as u128) as u32;
    SendWrapper::new(TimeoutFuture::new(millis))
}

/// Mirrors `tokio::time::error::Elapsed`
#[derive(Debug)]
pub(crate) struct Elapsed(());

/// Mirrors `tokio::time::timeout`
pub(crate) async fn timeout<F: Future>(
    duration: Duration,
    future: F,
) -> Result<F::Output, Elapsed> {
    match select(pin!(future), sleep(duration)).await {
        Either::Left((output, _)) => Ok(output),
        Either::Right(_) => Err(Elapsed(())),
    }
}

/// Mirrors `tokio::task::spawn_blocking`.
pub(crate) async fn spawn_blocking<F: FnOnce() -> R, R>(f: F) -> Result<R, Infallible> {
    Ok(f())
}
