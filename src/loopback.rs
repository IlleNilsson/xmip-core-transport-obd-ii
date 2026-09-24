//! Both ends of one OBD-II exchange on this machine (ADR-0051).
//!
//! A tester and an ECU, two nodes on a fresh simulated bus per round,
//! each end an ISO-TP session of its own: the tester's request
//! reaches the ECU, and its answer comes back. The far end
//! is the tester, polling the parameter and taking the data as the
//! Stream; the near end is the ECU, answering the one request with the
//! payload. The two ends need two threads, so the capability's `round`
//! drives it.

use std::sync::Arc;

use can_bus::Bus;
use iso_tp::IsoTpTransport;
use iso_tp::loopback::Session;
use sdk::broadcast::Medium;
use transport::error::Result;
use transport::loopback::{FarEnd, LOOPBACK_TIMEOUT, Loopback};

use crate::pid::{ECU, FUNCTIONAL, code};
use crate::{ObdTransport, Transport};

impl ObdTransport {
    /// Both ends on this machine: an ECU whose far end is a tester, the
    /// two nodes on a fresh simulated bus per round, the
    /// loopback timeout on both. The link this instance itself holds
    /// carries nothing; every round stands up its own.
    #[must_use]
    pub fn loopback() -> Self {
        let idle: Arc<dyn Bus> = Arc::new(Medium::new("loopback").node());
        let link =
            IsoTpTransport::new(Arc::clone(&idle), idle, ECU).timing_out_after(LOOPBACK_TIMEOUT);
        Self::new(link)
    }

    /// The ECU's end of the session at `address`.
    fn ecu_end(&self, address: &str) -> Result<Self> {
        let session = self.standing.session(address)?;
        let link = IsoTpTransport::new(Arc::clone(&session.ecu), session.ecu, ECU)
            .timing_out_after(LOOPBACK_TIMEOUT);
        Ok(Self {
            link,
            pid: self.pid,
            ecu: Arc::clone(&self.ecu),
            standing: self.standing.clone(),
        })
    }
}

impl Loopback for ObdTransport {
    /// The ISO-TP ceiling less the response header: the fact ISO 15765-4
    /// states about one message and SAE J1979 about its header.
    fn ceiling(&self) -> Option<usize> {
        Some(self.pid.ceiling())
    }

    /// A tester waiting to poll its one parameter. It owns the session:
    /// the address is forgotten once the data is taken.
    fn far_end(&self) -> Result<Box<dyn FarEnd>> {
        let session = Session::fresh();
        let link = IsoTpTransport::new(
            Arc::clone(&session.tester),
            Arc::clone(&session.tester),
            FUNCTIONAL,
        )
        .timing_out_after(LOOPBACK_TIMEOUT);
        let end = Self {
            link,
            pid: self.pid,
            ecu: Arc::clone(&self.ecu),
            standing: self.standing.clone(),
        };
        let address = self.standing.stand("obd", session);
        Ok(self.standing.far_end(address, move || end.poll(end.pid)))
    }

    /// The ECU answers the tester's one request with `payload`.
    fn send_to(&self, address: &str, payload: &[u8]) -> Result<()> {
        let ecu = self.ecu_end(address)?;
        ecu.send(&ecu.pid.origin("loopback"), payload)
    }

    /// No socket to poke. A tester whose ECU was refused is answered
    /// negatively, so it is judged now rather than at its deadline.
    fn unblock(&self, address: &str) {
        if let Ok(ecu) = self.ecu_end(address) {
            let refusal = ecu.pid.negative(code::REQUEST_OUT_OF_RANGE);
            drop(ecu.link.deliver(&refusal));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;
    use transport::payload::{edge_payloads, patterned};

    use crate::{Ecu, Pid};

    #[test]
    fn the_loopback_returns_the_edge_payloads_whole_and_refuses_over_the_brim() {
        let loopback = ObdTransport::loopback();
        let brim = loopback.pid.ceiling();
        let mut edges = edge_payloads();
        edges.push(("the brim", patterned(brim)));
        for (name, bytes) in edges {
            let arrived = loopback
                .round(&bytes)
                .unwrap_or_else(|error| panic!("{name}: {error}"));
            assert_eq!(arrived.bytes, bytes, "{name}");
            assert_eq!(arrived.origin_uri, "obd://loopback/0x09/0x0a", "{name}");
        }
        assert_eq!(Loopback::ceiling(&loopback), Some(4092));
        assert!(loopback.refuses(b"x").is_none());
        let started = Instant::now();
        let error = loopback
            .round(&patterned(brim + 1))
            .expect_err("one over the brim");
        assert!(error.message.starts_with("send failed:"), "{error}");
        assert!(
            started.elapsed() < LOOPBACK_TIMEOUT,
            "a refused send is judged, never waited on"
        );
        assert!(loopback.standing.is_empty(), "a taken session is forgotten");
    }

    #[test]
    fn a_current_data_parameter_polls_the_same_way() {
        let rpm = Pid::current(0x0c);
        let loopback = ObdTransport::loopback().at(rpm);
        let arrived = loopback.round(&[0x1a, 0xf8]).expect("rpm");
        assert_eq!(arrived.bytes, [0x1a, 0xf8]);
        assert_eq!(arrived.origin_uri, "obd://loopback/0x01/0x0c");
        assert_eq!(Loopback::ceiling(&loopback), Some(4093));
        let bitmap = ObdTransport::loopback()
            .at(Pid::current(0x00))
            .answering_as(Ecu::new().answering(rpm, &[0, 0]));
        let arrived = bitmap.round(b"ignored").expect("the bitmap is computed");
        assert_eq!(arrived.bytes, [0x00, 0x10, 0x00, 0x00]);
    }

    #[test]
    fn a_session_that_was_not_stood_up_is_refused_at_once() {
        let loopback = ObdTransport::loopback();
        let error = loopback
            .send_to("obd://loopback/0", b"x")
            .expect_err("no session");
        assert!(error.message.contains("not a session"), "{error}");
        loopback.unblock("obd://loopback/0");
    }
}
