//! Both ends of one OBD-II exchange on this machine (ADR-0051).
//!
//! A tester and an ECU on a fresh pair of directed loopback buses per
//! round, each end an ISO-TP session of its own: the tester's request
//! crosses one bus, the ECU's answer comes back on the other. The far end
//! is the tester, polling the parameter and taking the data as the
//! Stream; the near end is the ECU, answering the one request with the
//! payload. The two ends need two threads, so the capability's `round`
//! drives it.

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, PoisonError};

use can_bus::{Bus, Loopback as LoopbackBus};
use iso_tp::IsoTpTransport;
use transport::Arrived;
use transport::error::{Result, protocol_error};
use transport::loopback::{FarEnd, LOOPBACK_TIMEOUT, Loopback};

use crate::pid::{ECU, FUNCTIONAL, code};
use crate::{ObdTransport, Transport};

/// The two directed buses of one loopback session: the tester transmits on
/// `to_ecu` and reads `to_tester`, the ECU the other way round.
#[derive(Clone)]
pub(crate) struct Session {
    to_ecu: Arc<dyn Bus>,
    to_tester: Arc<dyn Bus>,
}

/// The sessions a loopback has stood up and not yet taken, by address. A
/// fresh pair of buses per round, so rounds driven at once from several
/// threads never read each other's frames.
pub(crate) type Standing = Arc<Mutex<HashMap<String, Session>>>;

/// Numbers the sessions, so each address names one.
static NEXT_SESSION: AtomicU64 = AtomicU64::new(1);

impl ObdTransport {
    /// Both ends on this machine: an ECU whose far end is a tester, the
    /// two on a fresh pair of directed loopback buses per round, the
    /// loopback timeout on both. The link this instance itself holds
    /// carries nothing; every round stands up its own.
    #[must_use]
    pub fn loopback() -> Self {
        let idle: Arc<dyn Bus> = Arc::new(LoopbackBus::new());
        let link =
            IsoTpTransport::new(Arc::clone(&idle), idle, ECU).timing_out_after(LOOPBACK_TIMEOUT);
        Self::new(link)
    }

    /// The ECU's end of the session at `address`.
    fn ecu_end(&self, address: &str) -> Result<Self> {
        let session = self
            .standing
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .get(address)
            .cloned()
            .ok_or_else(|| protocol_error(format!("{address} is not a session stood up here")))?;
        let link = IsoTpTransport::new(session.to_tester, session.to_ecu, ECU)
            .timing_out_after(LOOPBACK_TIMEOUT);
        Ok(Self {
            link,
            pid: self.pid,
            ecu: Arc::clone(&self.ecu),
            standing: Arc::clone(&self.standing),
        })
    }
}

/// A tester waiting to poll its one parameter. It owns the session: the
/// address is forgotten once the data is taken.
struct Polling {
    end: ObdTransport,
    address: String,
}

impl FarEnd for Polling {
    fn address(&self) -> &str {
        &self.address
    }

    fn take_one(self: Box<Self>) -> Result<Arrived> {
        let taken = self.end.poll(self.end.pid);
        self.end
            .standing
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .remove(&self.address);
        taken
    }
}

impl Loopback for ObdTransport {
    /// The ISO-TP ceiling less the response header: the fact ISO 15765-4
    /// states about one message and SAE J1979 about its header.
    fn ceiling(&self) -> Option<usize> {
        Some(self.pid.ceiling())
    }

    fn far_end(&self) -> Result<Box<dyn FarEnd>> {
        let session = Session {
            to_ecu: Arc::new(LoopbackBus::new()),
            to_tester: Arc::new(LoopbackBus::new()),
        };
        let link = IsoTpTransport::new(
            Arc::clone(&session.to_ecu),
            Arc::clone(&session.to_tester),
            FUNCTIONAL,
        )
        .timing_out_after(LOOPBACK_TIMEOUT);
        let address = format!(
            "obd://loopback/{}",
            NEXT_SESSION.fetch_add(1, Ordering::Relaxed)
        );
        self.standing
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .insert(address.clone(), session);
        Ok(Box::new(Polling {
            end: Self {
                link,
                pid: self.pid,
                ecu: Arc::clone(&self.ecu),
                standing: Arc::clone(&self.standing),
            },
            address,
        }))
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

    use crate::{Ecu, Pid};

    /// `len` bytes that a truncation, a reorder or a duplicate would change.
    fn patterned(len: usize) -> Vec<u8> {
        (0..len)
            .map(|at| u8::try_from((at * 31 + at / 251) % 256).unwrap_or(0))
            .collect()
    }

    #[test]
    fn the_loopback_returns_the_edge_payloads_whole_and_refuses_over_the_brim() {
        let loopback = ObdTransport::loopback();
        let brim = loopback.pid.ceiling();
        let edges: [(&str, Vec<u8>); 7] = [
            ("empty", Vec::new()),
            ("one byte", vec![0x2a]),
            ("every byte", (0..=255).collect()),
            ("nul run", vec![0; 512]),
            ("high bytes", vec![0xff; 512]),
            ("crlf storm", b"\r\n".repeat(400)),
            ("the brim", patterned(brim)),
        ];
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
        assert!(
            loopback.standing.lock().expect("lock").is_empty(),
            "a taken session is forgotten"
        );
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
