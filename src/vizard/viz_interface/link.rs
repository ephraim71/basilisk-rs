//! ZeroMQ sockets of [`VizInterface`](super::VizInterface): the broadcast
//! (publisher) socket and the two-way (requester) socket that runs Vizard in
//! lockstep with the simulation.

/// How long \[ms\] a closing socket keeps trying to deliver queued frames.
/// Upstream never closes its sockets; an unbounded linger would hang the
/// drop of a module whose Vizard has gone away.
const LINGER_MS: i32 = 1000;

/// Upstream's publisher socket (`ZMQ_PUB`), bound at `address`.
pub(super) struct Broadcaster {
    socket: zmq::Socket,
    endpoint: String,
}

impl Broadcaster {
    /// Binds a publisher at `address` (`protocol://host:port`; a port of `*`
    /// picks a free one).
    pub(super) fn bind(address: &str) -> Result<Self, zmq::Error> {
        let socket = zmq::Context::new().socket(zmq::PUB)?;
        socket.set_linger(LINGER_MS)?;
        socket.bind(address)?;
        let endpoint = socket
            .get_last_endpoint()?
            .unwrap_or_else(|_| address.to_string());
        Ok(Self { socket, endpoint })
    }

    /// The address the socket is bound at, with any `*` port resolved.
    pub(super) fn endpoint(&self) -> &str {
        &self.endpoint
    }

    /// Publishes `payload` under the `header` frame.
    pub(super) fn send(&self, header: &str, payload: &[u8]) -> Result<(), zmq::Error> {
        self.socket.send(header, zmq::SNDMORE)?;
        self.socket.send(payload, 0)
    }
}

/// Upstream's requester socket (`ZMQ_REQ`), connected to a two-way Vizard.
///
/// A requester must alternate sending and receiving. Upstream ignores the
/// error a receive out of turn returns; this tracks whether a reply is
/// outstanding instead.
pub(super) struct VizardLink {
    socket: zmq::Socket,
    awaiting_reply: bool,
}

impl VizardLink {
    /// Connects to Vizard at `address` and shakes hands as upstream `Reset`
    /// does: `PING`, wait for Vizard's answer, `PING` again. The answer to
    /// the second ping is collected by the next simulation update.
    ///
    /// Blocks until Vizard answers.
    pub(super) fn connect(address: &str) -> Result<Self, zmq::Error> {
        let socket = zmq::Context::new().socket(zmq::REQ)?;
        socket.set_linger(LINGER_MS)?;
        socket.connect(address)?;
        let mut link = Self {
            socket,
            awaiting_reply: false,
        };
        link.send("PING")?;
        link.receive()?;
        link.send("PING")?;
        Ok(link)
    }

    /// Sends a single-frame request.
    pub(super) fn send(&mut self, request: &str) -> Result<(), zmq::Error> {
        self.socket.send(request, 0)?;
        self.awaiting_reply = true;
        Ok(())
    }

    /// Receives every frame of the reply.
    pub(super) fn receive(&mut self) -> Result<Vec<Vec<u8>>, zmq::Error> {
        let frames = self.socket.recv_multipart(0)?;
        self.awaiting_reply = false;
        Ok(frames)
    }

    /// Discards the reply to the previous request, if one is outstanding.
    pub(super) fn discard_pending_reply(&mut self) -> Result<(), zmq::Error> {
        if self.awaiting_reply {
            self.receive()?;
        }
        Ok(())
    }

    /// Upstream's `SIM_UPDATE` request: the header, two empty frames, then
    /// the serialized `VizMessage`.
    pub(super) fn send_sim_update(&mut self, payload: &[u8]) -> Result<(), zmq::Error> {
        self.socket.send("SIM_UPDATE", zmq::SNDMORE)?;
        self.socket.send("", zmq::SNDMORE)?;
        self.socket.send("", zmq::SNDMORE)?;
        self.socket.send(payload, 0)?;
        self.awaiting_reply = true;
        Ok(())
    }
}
