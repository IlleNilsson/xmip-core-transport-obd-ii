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
//! **A receive is a poll, which consumes nothing at the ECU**, so its
//! verdict has nothing to tell it, whichever it is: a cycle that did not
//! complete loses nothing, and the next poll asks again.
//!
//! The origin URI names the parameter that was polled:
//! `obd://<bus>/0x<mode>/0x<pid>`.
//!
//! **Built with its `declaration` feature alone** the crate is
//! [`declaration`].

#[cfg(feature = "declaration")]
pub mod declaration;
#[cfg(feature = "default")]
pub mod ecu;
#[cfg(feature = "default")]
pub mod loopback;
#[cfg(feature = "default")]
pub mod pid;

#[cfg(feature = "default")]
use std::sync::Arc;

#[cfg(feature = "default")]
use can_bus::loopback::Session;
#[cfg(feature = "default")]
use codec::hex::prefixed_number;
#[cfg(feature = "default")]
use iso_tp::IsoTpTransport;
#[cfg(feature = "default")]
use net::{Target, ceiling};
#[cfg(feature = "default")]
use transport::error::{Result, protocol_error};
#[cfg(feature = "default")]
use transport::standing::Standing;
#[cfg(feature = "default")]
use transport::{Acknowledgement, Arrived, Configured, Directions, Taken, Transport};
#[cfg(feature = "default")]
use xcore::settings::{Read, Settings};

#[cfg(feature = "declaration")]
pub use declaration::{ECU, FUNCTIONAL, VEHICLE_INFORMATION};
#[cfg(feature = "default")]
pub use ecu::Ecu;
#[cfg(feature = "default")]
pub use pid::{CURRENT_DATA, Pid, code};

/// The parameter a Location polls or answers unless told otherwise: the
/// ECU name, information type `0a`, the one vehicle information item a
/// manufacturer fills freely.
#[cfg(feature = "default")]
const DEFAULT_PID: Pid = Pid::information(declaration::ECU_NAME);

/// One end of a diagnostic exchange: a tester when it receives, an ECU
/// when it sends.
#[cfg(feature = "default")]
#[derive(Clone)]
pub struct ObdTransport {
    link: IsoTpTransport,
    pid: Pid,
    ecu: Arc<Ecu>,
    standing: Standing<Session>,
}

#[cfg(feature = "default")]
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

    /// Ask the ECU for `pid` and take the data it answers with, whole.
    ///
    /// # Errors
    /// A negative response, an answer to another parameter, or a link that
    /// failed.
    pub fn poll(&self, pid: Pid) -> Result<Taken> {
        self.link.deliver(&pid.request())?;
        let answer = self.link.collect()?;
        let data = pid.parse_response(&answer.bytes)?;
        let mut polled = Taken::new(pid.origin(iso_tp::bus_of(&answer.origin_uri)), data);
        polled.observed = answer.observed;
        Ok(polled)
    }

    /// Answer one request as the ECU.
    ///
    /// # Errors
    /// A tester that never asks, a link that failed, or data no response
    /// carries.
    fn answer_one(&self) -> Result<()> {
        let request = self.link.collect()?;
        if request.bytes.is_empty() {
            return Err(protocol_error("the tester hung up"));
        }
        self.link.deliver(&self.ecu.answer(&request.bytes)?)
    }

    /// `obd://<bus>/0x<mode>/0x<pid>`: what `target` overrides of this
    /// end's parameter.
    fn addressed(&self, target: &str) -> Result<Pid> {
        let Some((_, path)) =
            Target::under(Self::SCHEMES, target).map(|named| (named.authority(), named.path()))
        else {
            return Ok(self.pid);
        };
        if path.is_empty() {
            return Ok(self.pid);
        }
        let (mode, pid) = path
            .split_once('/')
            .and_then(|(mode, pid)| Some((prefixed_number(mode).ok()?, prefixed_number(pid).ok()?)))
            .ok_or_else(|| protocol_error(format!("{path} is not a mode and a parameter")))?;
        Ok(Pid { mode, pid })
    }
}

#[cfg(feature = "default")]
impl Transport for ObdTransport {
    fn name(&self) -> &'static str {
        "obd-ii"
    }

    fn directions(&self) -> Directions {
        Directions::BOTH
    }

    fn arrivals(&self) -> transport::Arrivals {
        transport::Arrivals::Ordered("a poll reads again what is not yet told")
    }

    fn waiting(&self) -> transport::Waiting {
        transport::Waiting::Polled("a poll asks the ECU once and returns its answer")
    }

    /// Poll as the tester: one Stream per parameter, whole. The verdict has
    /// nothing to tell the ECU, whichever it is: a poll consumes nothing, so
    /// a cycle that did not complete loses nothing — the next poll asks
    /// again.
    fn receive_up_to(&self, _most: usize) -> Result<Vec<Arrived>> {
        let polled = self.poll(self.pid)?;
        Ok(vec![
            Arrived::whole(
                polled.origin_uri,
                polled.bytes,
                Acknowledgement::unconsumed(),
            )
            .scheduled()
            .observing_all(polled.observed),
        ])
    }

    fn sending(&self) -> transport::Sending {
        transport::Sending::Gathered("one OBD-II request carries the whole body")
    }

    /// Answer one request as the ECU, `bytes` being the parameter's data;
    /// `target` may name the parameter.
    fn send_from(&self, target: &str, outgoing: transport::Outgoing<'_>) -> Result<()> {
        let bytes: &[u8] = &outgoing.gather()?;
        let pid = self.addressed(target)?;
        ceiling::within(bytes.len(), pid.ceiling(), "one OBD-II response carries")?;
        self.ecu.set(pid, bytes);
        self.answer_one()
    }
}

#[cfg(feature = "default")]
impl Configured for ObdTransport {
    const SETTINGS: &'static Settings = declaration::SETTINGS;
    const SCHEMES: &'static [&'static str] = declaration::SCHEMES;

    /// The ISO-TP link's: a peer is waited on as the link waits.
    fn timeout_kept(&mut self) -> Option<&mut dyn transport::configured::TimeoutKept> {
        self.link.timeout_kept()
    }

    /// On a Location read for both sides, the tester's identifier is the
    /// one transmitted under.
    fn configured(address: &str, settings: &Read) -> Result<Self> {
        Ok(Self::on(can_bus::open_bus(address)?, settings))
    }
}

#[cfg(feature = "default")]
impl ObdTransport {
    /// The tester or the ECU a Location's settings build on `bus`: the bus
    /// its address names, or a simulated one a test gives it.
    pub(crate) fn on(bus: Arc<dyn can_bus::Bus>, settings: &Read) -> Self {
        // The declaration holds every integer within its range.
        let byte = |name| u8::try_from(settings.integer(name)).unwrap_or_default();
        let id = settings
            .optional_integer("request_id")
            .or_else(|| settings.optional_integer("response_id"))
            .map_or(ECU, |id| u32::try_from(id).unwrap_or_default());
        let link = IsoTpTransport::new(Arc::clone(&bus), bus, id);
        Self::new(link).at(Pid {
            mode: byte("mode"),
            pid: byte("pid"),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;
    use xcore::settings::Applies;

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

    // Built on a simulated bus by its own constructor, never by a name a
    // build with `socketcan` hands the kernel on Linux (another crate of the
    // estate's one workspace turns it on).
    #[test]
    fn obd_ii_declares_its_settings_and_reads_through_them() {
        use xcore::settings::Given;
        let on = |side, given: &[(String, Given)]| {
            let read = ObdTransport::SETTINGS.read(side, given).expect("read");
            ObdTransport::on(can_bus::simulated("vcan0"), &read).finished(&read, side)
        };
        assert_eq!(ObdTransport::SETTINGS.problems(), Vec::<String>::new());
        let given = [
            ("mode".to_string(), Given::Integer(1)),
            ("pid".to_string(), Given::Integer(0x0c)),
            ("timeout".to_string(), Given::Text("250ms".to_string())),
        ];
        let tester = on(Applies::Receive, &given);
        assert_eq!(tester.pid, Pid::current(0x0c));
        // Left out, the Location waits the default it declares: the ISO-TP
        // link's own, which the link waits when given none.
        let declared = ObdTransport::SETTINGS;
        let unsaid = declared.read(Applies::Receive, &given[..2]).expect("read");
        let timeout = unsaid.optional_duration(transport::configured::TIMEOUT);
        assert_eq!(timeout, Some(iso_tp::DEFAULT_TIMEOUT));
        let ecu = on(Applies::Send, &[]);
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
        let polling = tester.clone().at(Pid::current(0x0c));
        let arrived = polling.receive().expect("rpm").remove(0);
        assert!(arrived.defers(), "a poll consumes nothing: nothing to lose");
        let arrived = arrived.taken().expect("taken");
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
    }
}
