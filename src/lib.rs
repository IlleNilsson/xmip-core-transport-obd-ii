#![forbid(unsafe_code)]

//! Streams polled from an emissions ECU, the way SAE J1979 asks for them.
//!
//! OBD-II is the port under every dashboard since 1996: a tester asks a
//! mode and a parameter at the functional address `0x7df`, and the ECU
//! answers from `0x7e8` with the mode plus `0x40`, the parameter and the
//! data. Mode `01` is current data, mode `09` vehicle information. A
//! Receive Location polls one parameter and its data arrives as a Stream; a
//! Send Location answers one as the ECU would, the Stream it is given
//! being the data. One ISO-TP message carries a response, so a parameter's
//! data is at most the message's ceiling less the response header — the
//! ceiling, and a fact of the protocol.
//!
//! The carrier is [`iso_tp`](iso_tp): each request and each response is
//! one ISO-TP message. A tester and an ECU are two nodes on one bus — in
//! process, the SDK's simulated one — so they round-trip with no hardware,
//! which is what [`ObdTransport::loopback`] stands up (ADR-0051).
//!
//! The origin URI names the parameter that was polled:
//! `obd://<bus>/0x<mode>/0x<pid>`.

pub mod ecu;
pub mod loopback;
pub mod pid;

use std::sync::Arc;

use can_bus::loopback::Session;
use iso_tp::IsoTpTransport;
use transport::ceiling;
use transport::error::{Result, protocol_error};
use transport::standing::Standing;
use transport::{Arrived, Configured, Directions, Transport};
use xcore::settings::{Applies, Fixed, Kind, Presence, Read, Setting, Settings};

pub use ecu::Ecu;
pub use pid::{CURRENT_DATA, ECU, FUNCTIONAL, Pid, VEHICLE_INFORMATION, code};

/// The parameter a Location polls or answers unless told otherwise: the
/// ECU name, information type `0a`, the one vehicle information item a
/// manufacturer fills freely.
pub const DEFAULT_PID: Pid = Pid::information(0x0a);

/// One end of a diagnostic exchange: a tester when it receives, an ECU
/// when it sends.
#[derive(Clone)]
pub struct ObdTransport {
    link: IsoTpTransport,
    pid: Pid,
    ecu: Arc<Ecu>,
    standing: Standing<Session>,
}

impl ObdTransport {
    /// An exchange over `link`, polling and answering [`DEFAULT_PID`] as an
    /// ECU holding nothing.
    #[must_use]
    pub fn new(link: IsoTpTransport) -> Self {
        Self {
            link,
            pid: DEFAULT_PID,
            ecu: Arc::new(Ecu::new()),
            standing: Standing::default(),
        }
    }

    /// Poll and answer `pid`.
    #[must_use]
    pub const fn at(mut self, pid: Pid) -> Self {
        self.pid = pid;
        self
    }

    /// Answer as `ecu` when sending.
    #[must_use]
    pub fn answering_as(mut self, ecu: Ecu) -> Self {
        self.ecu = Arc::new(ecu);
        self
    }

    /// The ECU this end answers as.
    #[must_use]
    pub fn ecu(&self) -> &Ecu {
        &self.ecu
    }

    /// Ask the ECU for `pid` and take the data it answers with.
    ///
    /// # Errors
    /// A negative response, an answer to another parameter, or a link that
    /// failed.
    pub fn poll(&self, pid: Pid) -> Result<Arrived> {
        self.link.deliver(&pid.request())?;
        let answer = self.link.collect()?;
        let data = pid.parse_response(&answer.bytes)?;
        Ok(Arrived::new(pid.origin(&bus_of(&answer.origin_uri)), data))
    }

    /// Answer one request as the ECU.
    ///
    /// # Errors
    /// A tester that never asks, a link that failed, or data no response
    /// carries.
    pub fn answer_one(&self) -> Result<()> {
        let request = self.link.collect()?;
        if request.bytes.is_empty() {
            return Err(protocol_error("the tester hung up"));
        }
        self.link.deliver(&self.ecu.answer(&request.bytes)?)
    }

    /// `obd://<bus>/0x<mode>/0x<pid>`: what `target` overrides of this
    /// end's parameter.
    fn addressed(&self, target: &str) -> Result<Pid> {
        let Some((_, path)) = transport::socket::target("obd", target) else {
            return Ok(self.pid);
        };
        if path.is_empty() {
            return Ok(self.pid);
        }
        let (mode, pid) = path
            .split_once('/')
            .and_then(|(mode, pid)| Some((hex(mode)?, hex(pid)?)))
            .ok_or_else(|| protocol_error(format!("{path} is not a mode and a parameter")))?;
        Ok(Pid { mode, pid })
    }
}

fn hex(text: &str) -> Option<u8> {
    u8::from_str_radix(text.strip_prefix("0x")?, 16).ok()
}

/// The bus an ISO-TP origin names.
fn bus_of(origin: &str) -> String {
    origin
        .strip_prefix("isotp://")
        .and_then(|rest| rest.split('/').next())
        .unwrap_or("isotp")
        .to_string()
}

impl Transport for ObdTransport {
    fn name(&self) -> &'static str {
        "obd-ii"
    }

    fn directions(&self) -> Directions {
        Directions::BOTH
    }

    /// Poll as the tester: one Stream per parameter.
    fn receive(&self) -> Result<Vec<Arrived>> {
        Ok(vec![self.poll(self.pid)?])
    }

    /// Answer one request as the ECU, `bytes` being the parameter's data;
    /// `target` may name the parameter.
    fn send(&self, target: &str, bytes: &[u8]) -> Result<()> {
        let pid = self.addressed(target)?;
        ceiling::within(bytes.len(), pid.ceiling(), "one OBD-II response carries")?;
        self.ecu.set(pid, bytes);
        self.answer_one()
    }
}

/// A byte of a parameter: its mode or its number.
const BYTE: Kind = Kind::Integer {
    minimum: 0,
    maximum: 255,
};

/// A CAN identifier, extended above `0x7ff`.
const CAN_ID: Kind = Kind::Integer {
    minimum: 0,
    maximum: 0x1fff_ffff,
};

impl Configured for ObdTransport {
    /// The address is the CAN interface, `can0`, the ISO-TP link rides. A
    /// Receive Location is the tester and transmits at the functional
    /// address; a Send Location is the ECU and answers from its own.
    const SETTINGS: &'static Settings = &Settings {
        technology: env!("CARGO_PKG_NAME"),
        settings: &[
            Setting {
                name: "mode",
                kind: BYTE,
                presence: Presence::Default(Fixed::Integer(DEFAULT_PID.mode as i64)),
                meaning: "The mode of the parameter polled or answered: 1 current data, 9 \
                          vehicle information.",
                applies: Applies::Both,
            },
            Setting {
                name: "pid",
                kind: BYTE,
                presence: Presence::Default(Fixed::Integer(DEFAULT_PID.pid as i64)),
                meaning: "The number of the parameter polled or answered within its mode.",
                applies: Applies::Both,
            },
            Setting {
                name: "request_id",
                kind: CAN_ID,
                presence: Presence::Default(Fixed::Integer(FUNCTIONAL as i64)),
                meaning: "The CAN identifier a tester asks under.",
                applies: Applies::Receive,
            },
            Setting {
                name: "response_id",
                kind: CAN_ID,
                presence: Presence::Default(Fixed::Integer(ECU as i64)),
                meaning: "The CAN identifier an ECU answers from.",
                applies: Applies::Send,
            },
            Setting {
                name: "timeout",
                kind: Kind::Duration,
                presence: Presence::Optional,
                meaning: "How long a peer that stops mid-message is waited on; the ISO-TP \
                          link's own when left out.",
                applies: Applies::Both,
            },
        ],
    };

    /// On a Location read for both sides, the tester's identifier is the
    /// one transmitted under.
    fn configured(address: &str, settings: &Read) -> Result<Self> {
        // The declaration holds every integer within its range.
        let byte = |name| u8::try_from(settings.integer(name)).unwrap_or_default();
        let id = settings
            .optional_integer("request_id")
            .or_else(|| settings.optional_integer("response_id"))
            .map_or(ECU, |id| u32::try_from(id).unwrap_or_default());
        let bus = can_bus::open_bus(address)?;
        let link = IsoTpTransport::new(Arc::clone(&bus), bus, id);
        let link = match settings.optional_duration("timeout") {
            Some(timeout) => link.timing_out_after(timeout),
            None => link,
        };
        Ok(Self::new(link).at(Pid {
            mode: byte("mode"),
            pid: byte("pid"),
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    /// A tester and an ECU, nodes on one simulated bus, on this thread: what the
    /// tester asks sits on the bus until the ECU is asked to answer.
    fn pair() -> (ObdTransport, ObdTransport) {
        let Session {
            near: at_tester,
            far: at_ecu,
        } = Session::fresh();
        let quick = Duration::from_millis(20);
        let tester = IsoTpTransport::new(Arc::clone(&at_tester), at_tester, FUNCTIONAL)
            .timing_out_after(quick);
        let ecu = IsoTpTransport::new(Arc::clone(&at_ecu), at_ecu, ECU).timing_out_after(quick);
        (ObdTransport::new(tester), ObdTransport::new(ecu))
    }

    #[test]
    fn obd_ii_declares_its_settings_and_reads_through_them() {
        use xcore::settings::Given;
        assert_eq!(ObdTransport::SETTINGS.problems(), Vec::<String>::new());
        let given = [
            ("mode".to_string(), Given::Integer(1)),
            ("pid".to_string(), Given::Integer(0x0c)),
            ("timeout".to_string(), Given::Text("250ms".to_string())),
        ];
        let tester = ObdTransport::open("vcan0", Applies::Receive, &given).expect("built");
        assert_eq!(tester.pid, Pid::current(0x0c));
        let ecu = ObdTransport::open("vcan0", Applies::Send, &[]).expect("built");
        assert_eq!(ecu.pid, DEFAULT_PID);
        let wrong = [("request_id".to_string(), Given::Integer(0x7df))];
        let Err(refused) = ObdTransport::open("vcan0", Applies::Send, &wrong) else {
            panic!("request_id is the tester's");
        };
        assert!(
            refused.message.contains("\"request_id\""),
            "{}",
            refused.message
        );
    }

    #[test]
    fn an_ecu_that_does_not_have_the_parameter_says_so() {
        let (tester, ecu) = pair();
        let rpm = Pid::current(0x0c);
        tester.link.deliver(&rpm.request()).expect("asking");
        ecu.answer_one().expect("answering");
        let answer = tester.link.collect().expect("the answer").bytes;
        let error = rpm.parse_response(&answer).expect_err("refused");
        assert!(error.message.contains("0x31"), "{error}");
        assert!(tester.poll(rpm).is_err(), "nobody answers");
        ecu.answer_one().expect("the request left waiting");
        tester.link.deliver(&[]).expect("hang up");
        let error = ecu.answer_one().expect_err("the tester hung up");
        assert!(error.message.contains("hung up"), "{error}");
    }

    #[test]
    fn a_send_sets_the_answer_and_answers_one_request() {
        let (tester, ecu) = pair();
        tester
            .link
            .deliver(&Pid::current(0x0c).request())
            .expect("asking");
        ecu.send("obd://can0/0x01/0x0c", &[0x1a, 0xf8])
            .expect("answering");
        let arrived = tester.poll(Pid::current(0x0c)).expect("rpm");
        assert_eq!(arrived.bytes, [0x1a, 0xf8]);
        assert_eq!(arrived.origin_uri, "obd://loopback/0x01/0x0c");
        assert_eq!(
            ecu.ecu().held(Pid::current(0x0c)).expect("held"),
            [0x1a, 0xf8]
        );
        let error = ecu
            .send("obd://can0/0x09/0x0a", &[0; 4093])
            .expect_err("over");
        assert!(error.message.contains("is over the 4092"), "{error}");
    }

    #[test]
    fn a_target_names_the_parameter_and_the_ends_name_themselves() {
        let (tester, _) = pair();
        assert_eq!(
            tester.addressed("obd://can0/0x09/0x02").expect("vin"),
            Pid::information(0x02)
        );
        assert_eq!(
            tester.addressed("obd://can0/").expect("default"),
            DEFAULT_PID
        );
        assert_eq!(tester.addressed("elsewhere").expect("default"), DEFAULT_PID);
        assert!(tester.addressed("obd://can0/0x09").is_err());
        assert!(tester.addressed("obd://can0/vin/0x02").is_err());
        assert_eq!(tester.name(), "obd-ii");
        assert!(tester.claims().is_none());
        assert!(tester.directions().receives() && tester.directions().sends());
        assert_eq!(tester.at(Pid::current(0x05)).pid, Pid::current(0x05));
        assert_eq!(bus_of("elsewhere"), "isotp");
    }
}
